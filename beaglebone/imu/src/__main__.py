import Adafruit_BBIO.UART as UART
import serial
import asyncio
import math
import time
import struct
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import IMUData
import bitstring

# Picocom - picocom -b 115200 /dev/ttyS4
# LCM_DEFAULT_URL="udpm://239.255.76.67:76?ttl=255" ./jarvis exec lcm_tools_echo IMUData "/imu_data"

# Calibration Matrix Values
# CREG MAG CAL1_1 0x0F
# CREG MAG CAL1_2 0x10
# CREG MAG CAL1_3 0x11
# CREG MAG CAL2_1 0x12
# CREG MAG CAL2_2 0x13
# CREG MAG CAL2_3 0x14
# CREG MAG CAL3_1 0x15
# CREG MAG CAL3_2 0x16
# CREG MAG CAL3_3 0x17

# In Matrix:
# 0x0F 0x10 0x11
# 0x12 0x13 0x14
# 0x15 0x16 0x17

# 0x18 X-Axis Mag Bias 
# 0x19 Y-Axis Mag Bias
# 0x1A Z-Axis Mag Bias

# Raw Mag Values
# Mag Raw XY: 0x5C
# Mag Raw Z : 0x5D
# Mag Raw TI: 0x5E



class IMU_Manager():
    # data for clibration
    calibration_matrix = [[0]*3]*3
    mag_offsets = [0]*3

    def __init__(self):
        UART.setup("UART4")

        # Mapping NMEA messages to their handlers
        self.NMEA_TAGS_MAPPER = {
            "PCHRS": self.pchrs_handler,
            "PCHRA": self.pchra_handler,
            "PCHRH": self.pchrh_handler
        }
        self.sleep = .01

    def __enter__(self):

        self.ser = serial.Serial(
            port='/dev/ttyS4',
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Closes serial connection to UM-7
        '''
        self.ser.close()

    def pchrs_handler(self, msg, imu_struct):
        # packet type can be either 0: gyro, 1: accel, 2: magnetometer
        # mag data is unit-nrom (unitless)
        try:
            arr = msg.split(",")

            

            # Checksum checking
            checksum = int(arr[6][1:3], 16)
            if(checksum != self.calc_checksum(msg)):
                # error in checksum
                raise ValueError("Failed Checksum")

            print(arr)
            packetType = arr[1]
            if (packetType == '0'):
                imu_struct.gyro_x_dps = float(arr[3])
                imu_struct.gyro_y_dps = float(arr[4])
                imu_struct.gyro_z_dps = float(arr[5])
            elif (packetType == '1'):
                imu_struct.accel_x_g = float(arr[3])
                imu_struct.accel_y_g = float(arr[4])
                imu_struct.accel_z_g = float(arr[5])
            elif (packetType == '2'):
                imu_struct.mag_x_uT = float(arr[3])
                imu_struct.mag_y_uT = float(arr[4])
                imu_struct.mag_z_uT = float(arr[5])
                bearing = -(math.atan2(float(arr[4]), float(arr[3])) * (180.0 / math.pi))
                if(bearing < 0):
                    bearing += 360
                imu_struct.bearing_deg = bearing
            else:
                # this shouldnt happen
                print("Passed")
                pass
        # fill with zeroes if something goes wrong.
        except Exception as a:
            print("Error with PCHRS handler")
            print(a.args)
            imu_struct.gyro_x_dps = 0
            imu_struct.gyro_y_dps = 0
            imu_struct.gyro_z_dps = 0
            imu_struct.accel_x_g = 0
            imu_struct.accel_y_g = 0
            imu_struct.accel_z_g = 0
            imu_struct.mag_x_uT = 0
            imu_struct.mag_y_uT = 0
            imu_struct.mag_z_uT = 0

    def pchra_handler(self, msg, imu_struct):
        pi = 3.14159265359
        try:
            arr = msg.split(",")

            # Checksum checking
            checksum = int(arr[5][1:3], 16)
            if(checksum != self.calc_checksum(msg)):
                # error in checksum
                raise ValueError("Failed Checksum")

            # raw values are in degrees, need to convert to radians
            imu_struct.roll_rad = float(arr[2]) * pi / 180
            imu_struct.pitch_rad = float(arr[3]) * pi / 180
            yaw = float(arr[4])
            if(yaw < 0):
                yaw += 360
            imu_struct.yaw_rad = yaw
            # fill with zeroes if something goes wrong
        except Exception as b:
            print("Error with PCHRA handler")
            print(b.args)
            imu_struct.roll_rad = 0
            imu_struct.pitch_rad = 0
            imu_struct.yaw_rad = 0

    def pchrh_handler(self, msg, imu_struct):
        try:
            arr = msg.split(",")

            print("Satalites being used: ", arr[2])
            print("Satalites being tracked: ", arr[3])
            print("HDOP: ", arr[4])
            print("Mode (0 is Euler Angle, 1 is quaternion): ", arr[5])
            print("COM (1 means it is transmitting too much): ", arr[6])
            print("Accel (0-1; high if not read correctly): ", arr[7])
            print("Gyro (0-1; high if not read correctly): ", arr[8])
            print("Mag (0-1; high if not read correctly): ", arr[9])
            print("GPS (0-1; high if not read correctly): ", arr[10])

        except Exception as b:
            print("Error with PCHRH handler")
            print(b.args)

    async def recieve(self, lcm):
            '''
            Reads from the rover IMU over serial connection.
            Attempts to read and proccess all supported NMEA tags
            at least once before publishing a new LCM message.
            Sleeps after publishing to
            allow time for handling incoming LCM messages
            '''
            imu = IMUData()
            error_counter = 0
            # Mark TXT as always seen because they are not necessary
            while True:
                try:
                    msg = str(self.ser.readline())
                    error_counter = 0
                except Exception as e:
                    if error_counter < self.max_error_count:
                        error_counter += 1
                        print(e)
                        await asyncio.sleep(self.sleep)
                        continue
                    else:
                        raise e

                match_found = False
                for tag, func in self.NMEA_TAGS_MAPPER.items():
                    if tag in msg:
                        match_found = True
                        try:
                            if(tag == "PCHRS"):
                                print(msg)
                                func(msg, imu)
                                lcm.publish('/imu_data', imu.encode())
                            if(tag == "PCHRA"):
                                print(msg)
                                func(msg, imu)
                                lcm.publish('/imu_data', imu.encode())
                            if(tag == "PCHRH"):
                                print(msg)
                                func(msg, imu)
                        except Exception as e:
                            print(e)
                            break

                    if not match_found:
                        if not msg:
                            print('Error decoding message stream: {}'.format(msg))

                await asyncio.sleep(self.sleep)

    # turns off registers that are outputting non-NMEA data
    def turnOffRegister(self, register):
        checksum = ord('s') + ord('n') + ord('p') + register + 0x80

        cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x80, register,
                      0x00, 0x00, 0x00, 0x00, checksum >> 8, checksum & 0xff]

        self.ser.write(cmd_buffer)

    # turns on the attidude and sensor nmea sentences to 1Hz
    def enable_nmea(self, register):
        checksum = ord('s') + ord('n') + ord('p') + register + 0x80 + 0x11

        cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x80, register,
                      0, 0x11, 0, 0, checksum >> 8, checksum & 0xff]

        self.ser.write(cmd_buffer)

        # health packet
        checksum = ord('s') + ord('n') + ord('p') + 0x06 + 0x80 + 0x01
        cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x80, 0x06,
                      0, 0x01, 0, 0, checksum >> 8, checksum & 0xff]

        self.ser.write(cmd_buffer)


    def get_calibrated(self, register):
        checksum = ord('s') + ord('n') + ord('p') + register
        cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x00, register,
                      checksum >> 8, checksum & 0xff]

        # sends reques for data
        self.ser.write(cmd_buffer)
        time.sleep(.5)

        # um7 sends back same message as request for data but with payload
        # containing IEEE 754 32bit number (= to python float)
        received = self.ser.readline()
        # print(received)
        # Filters the buffer looking for the has data packets and prints it
        run = True
        iterator = 0
        while (run):
            if(received[iterator:(iterator + 4)] == b'snp\x80'):
                # print(received[iterator:(iterator + 11)])
                run = False
                data = [0x00, 0x00, 0x00, 0x00]
                # What happens when a byte in received is empty? Does it get ignored or is it kept as all 0
                data = received[(iterator + 5):(iterator + 9)]
                # print(data)
                # plaincontent = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3])
                # print(plaincontent)
                val = struct.unpack('>f', data)
                print(val)
            iterator = iterator + 1

    # gets calibration matrix values as 32 bit IEEE f-point 
    def get_cal_vals(self, reg):
        checksum = ord('s') + ord('n') + ord('p') + reg
        cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x00, reg,
                      checksum >> 8, checksum & 0xff]
        
        self.ser.write(cmd_buffer)
        time.sleep(0.5)
        received = self.ser.readline()
        data = received.hex()[10:18]
        # print("data: \n", data)
        # data_f = struct.unpack('>f', struct.pack(">i", int(data, 16)))[0]
        # convert IEEE to float
        bits = bitstring.BitArray(hex = data)
        # print("bits: \n", bits)
        data_f = bits.float
        # print("cal_val: ", data_f)

        return data_f


    def get_raw(self, registerxy, registerz):
        # checksum = ord('s') + ord('n') + ord('p') + registerxy
        # cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x00, registerxy,
        #               checksum >> 8, checksum & 0xff]

        # batch cmd
        checksum = ord('s') + ord('n') + ord('p') + registerxy
        cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x48, registerxy,
                      checksum >> 8, checksum & 0xff]

        # sends reques for data
        self.ser.write(cmd_buffer)
        time.sleep(.01)
        # print("cmd_buff: \n", cmd_buffer)
        
        # Filters the buffer looking for the has data packets and prints it
        run = True
        iterator = 0

        data_x=0
        data_y=0
        data_z=0
        # Waits for snp packet to come through
        while (run):
            # um7 sends back same message as request for data but with payload
            # containing 2's complement data for mag
            received = self.ser.readline()
            print("received: ", received.hex())
            # received = self.ser.readline()
            if(received.hex()[0:8] == (b'snp\xC8').hex()):
                run = False
                # get data
                bits = bitstring.BitArray(hex=received.hex())
                if(bits[40:56].len > 0):
                    data_x = bits[40:56].unpack('int:16')
                else:
                    data_x = [0]
                if(bits[56:72].len > 0):
                    data_y = bits[56:72].unpack('int:16')
                else:
                    data_y = [0]
                if(bits[72:88].len > 0):
                    data_z = bits[72:88].unpack('int:16')
                else:
                    data_z = [0]
                # print("data_x: ", data_x)
                # print("data_y: ", data_y)
            if(iterator == 1000):
                print("No Data received, aborting search")
                run = False
            iterator = iterator + 1

        # now for the z value
        # checksum = ord('s') + ord('n') + ord('p') + registerz
        # cmd_buffer = [ord('s'), ord('n'), ord('p'), 0x00, registerz,
        #               checksum >> 8, checksum & 0xff]

        # self.ser.write(cmd_buffer)
        # time.sleep(0.01)
        # run = True
        # iterator = 0
        # while (run):
        #     received = self.ser.readline()
        #     if(received.hex()[0:10] == (b'snp\x80\x5D').hex()):
        #         run = False
        #         bits = bitstring.BitArray(hex=received.hex())
        #         if(bits[40:56].len > 0):
        #             data_z = bits[40:56].unpack('int:16')
        #         else:
        #             data_z = [0]
        #         # print("data_z: ", data_z)
        #     #if(iterator == 1000):
        #         #print("No Data Received, aborting search")
        #         #run = False
        #     iterator = iterator + 1

        return data_x, data_y, data_z

    # returns hex value of calculated checksum fromt the message
    # checksum is computed using XOR of every byte in the packet
    # between '$' and '*' noninclusive
    def calc_checksum(self, msg):
        c_checksum = 0
        for b in msg[3:-8]:
            c_checksum ^= ord(b)
        c_checksum = hex(c_checksum)
        c_checksum = (str(c_checksum))[2:4]
        c_checksum = int(c_checksum, 16)
        return c_checksum

    # Gets calibration values from registers - only need to run once at beginning
    def get_calibration_vals(self, calibration_matrix, mag_offsets):
        CAL_REG = [[0x0F, 0x10, 0x11], [0x12, 0x13, 0x14], [0x15, 0x16, 0x17]]
        BIAS_REG = [0x18, 0x19, 0x1A]

        # Gets calibration matrix values
        # Soft-Iron Calibration
        # for i in range(3):
        #    for j in range(3):
                # print("matrix coord: ", i, " ", j, " \n") 
        #        IMU_Manager.calibration_matrix[i][j] = self.get_cal_vals(CAL_REG[i][j])
        #manual cal_matrix
        IMU_Manager.calibration_matrix = [[0.1645294, 0.1290768, 0.00037273],
                                          [0,         0.7418654, -0.3292478],
                                          [0,         0,          0.0202561]]

        # Gets Magnetometer biases
        # Hard-Iron calibration
        # for i in range(3):
        #    IMU_Manager.mag_offsets[i] = self.get_cal_vals(BIAS_REG[i])
        #manual mag_offsets
        IMU_Manager.mag_offsets = [2, 106, -26]
    
    def calculate_bearing(self):
        
        # get raw values for mag
        data_xf, data_yf, data_zf = self.get_raw(0x5C, 0x5D)
        print("data: ", data_xf, " ", data_yf, " ", data_zf)
        #=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-
        # Magnetometer reads in unitless
        
        mag_x = data_xf[0] - IMU_Manager.mag_offsets[0]
        mag_y = data_yf[0] - IMU_Manager.mag_offsets[1]
        mag_z = data_zf[0] - IMU_Manager.mag_offsets[2]
        print("offsets: ", IMU_Manager.mag_offsets[0], " ", IMU_Manager.mag_offsets[1], " ", IMU_Manager.mag_offsets[2])
        print("cal_mat: ", IMU_Manager.calibration_matrix[0][1], " ", IMU_Manager.calibration_matrix[0][2])
        # Apply mag soft iron error compensation
        mag_calibrated_x = mag_x * IMU_Manager.calibration_matrix[0][0] + \
            mag_y * IMU_Manager.calibration_matrix[0][1] + mag_z * IMU_Manager.calibration_matrix[0][2]

        mag_calibrated_y = mag_x * IMU_Manager.calibration_matrix[1][0] + \
            mag_y * IMU_Manager.calibration_matrix[1][1] + mag_z * IMU_Manager.calibration_matrix[1][2]

        mag_calibrated_z = mag_x * IMU_Manager.calibration_matrix[2][0] + \
            mag_y * IMU_Manager.calibration_matrix[2][1] + mag_z * IMU_Manager.calibration_matrix[2][2]

        print("cal_xyz: ", mag_calibrated_x, " ", mag_calibrated_y, " ", mag_calibrated_z)
        # Bearing Calculation
        bearing = -(math.atan2(mag_calibrated_y, mag_calibrated_x) * (180.0 / math.pi))

        if (bearing < 0):
            bearing += 360

        print("bearing: ", bearing)

        # Adds to struct
        # imu_struct.mag_x_uT = float(mag_calibrated_x)
        # imu_struct.mag_y_uT = float(mag_calibrated_y)
        # imu_struct.mag_z_uT = float(mag_calibrated_z)

        #=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-





# end of class


def main():
    # Uses a context manager to ensure serial port released
    NMEA_RATE_REG = 0x07
    GYRO_PROC = [0x61, 0x62, 0x63]
    MAG_PROC = [0x69, 0x6A, 0x6B]
    ACCEL_PROC = [0x65, 0x66, 0x67]

    with IMU_Manager() as manager:
        # turns off registers that are outputting non-NMEA data
        l = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
        for reg in l:
            manager.turnOffRegister(reg)

        # spits out calibrated values from registers
        # for r in GYRO_PROC:
        #     print("GYRO_PROC")
        #     manager.get_calibrated(r)
        #     print("\n")
        # for r in ACCEL_PROC:
        #     print("ACCEL_PROC")
        #     manager.get_calibrated(r)
        #     print("\n")
        # for r in MAG_PROC:
        #     print("MAG_PROC")
        #     manager.get_calibrated(r)
        #     print("\n")

        manager.get_calibration_vals(manager.calibration_matrix, manager.mag_offsets)        
        while(True):
            manager.calculate_bearing()
        # print("RAW_ACCEL")
        # manager.get_raw(0x59, 0x5A)
        
        


        # print("RAW_MAG")
        # manager.get_raw(0x5C, 0x5D)

        manager.enable_nmea(NMEA_RATE_REG)

        lcm = aiolcm.AsyncLCM()

        run_coroutines(lcm.loop(), manager.recieve(lcm))


if __name__ == "__main__":
    main()
