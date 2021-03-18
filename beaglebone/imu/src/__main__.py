import Adafruit_BBIO.UART as UART
import serial
import struct
import time
import asyncio
from os import getenv
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import IMUData

class IMU_Manager():

    def __init__(self):
        UART.setup("UART4")

        # Mapping NMEA messages to their handlers
        self.NMEA_TAGS_MAPPER = {
            "PCHRS": self.pchrs_handler
        }

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
        arr = msg.split(",")
        
        # packet type can be either 0: gyro, 1: accel, 2: magnetometer
        packetType = arr[1]
        if (packetType == 0):
            imu_struct.gyro_x_dps = arr[3]
            imu_struct.gyro_y_dps = arr[4]
            imu_struct.gyro_z_dps = arr[5]
        elif (packetType == 1):
            imu_struct.accel_x_g = arr[3]
            imu_struct.accel_y_g = arr[4]
            imu_struct.accel_z_g = arr[5]
        elif (packetType == 2):
            imu_struct.mag_x_uT = arr[3]
            imu_struct.mag_y_uT = arr[3]
            imu_struct.mag_z_uT = arr[3]
        else:
            # this shouldnt happen
            pass


    async def recieve(self, lcm):
            '''
            Reads from the rover IMU over serial connection.
            Attempts to read and proccess all supported NMEA tags
            at least once before publishing a new LCM message.
            Sleeps after publishing to
            allow time for handling incoming LCM messages
            '''
            imu_struct = IMUData()
            error_counter = 0
            # Mark TXT as always seen because they are not necessary
            seen_tags = {tag: False if not tag == 'TXT' else True
                        for tag in self.NMEA_TAGS_MAPPER.keys()}
            while True:
                # Wait for all tags to be seen
                while (not all(seen_tags.values())):
                    try:
                        msg = str(self.ser.readline())
                        print(msg)
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
                                func(msg, imu_struct)
                                seen_tags[tag] = True
                            except Exception as e:
                                print(e)
                            break

                    if not match_found:
                        print('Error decoding message stream: {}'.format(msg))

                lcm.publish('/imu_data', imu_struct.encode())
                seen_tags = {tag: False if not tag == 'TXT' else True
                            for tag in self.NMEA_TAGS_MAPPER.keys()}
                await asyncio.sleep(self.sleep)

    # turns off registers that are outputting non-NMEA data
    def turnOffRegister(self, register):
        checksum = ord('s') + ord('n') + ord('p') + register + 0x80

        cmd_buffer = {ord('s'), ord('n'), ord('p'), 0x80, register,
                     0x00, 0x00, 0x00, 0x00, checksum >> 8, checksum & 0xff}
        
        ser.write(bytes(cmd_buffer))
    

    def enable_nmea(self, register):
        checksum = ord('s') + ord('n') + ord('p') + register + 0x80 + 0x01
        
        cmd_buffer = {ord('s'), ord('n'), ord('p'), 0x80, register,
                         0 , 0x01, 0, 0, checksum >> 8, checksum & 0xff}
        
        ser.write(bytes(cmd_buffer))

# end of class

def main():
    # Uses a context manager to ensure serial port released
    NMEA_RATE_REG = 0x07

    with IMU_Manager() as manager:
        # turns off registers that are outputting non-NMEA data
        l = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}
        for reg in l:
            manager.turnOffRegister(reg)
        
        manager.enable_nmea(NMEA_RATE_REG)

        lcm = aiolcm.AsyncLCM()

        run_coroutines(lcm.loop(), manager.recieve(lcm))


if __name__ == "__main__":
    main()




#https://learn.adafruit.com/setting-up-io-python-library-on-beaglebone-black/uart
#UART1 RX:P9_26 TX:P9_24 CTS:P9_20 RTS:P9_19
#UART2 RX:P9_22 TX:P9_21



#Binary packets Pg. 31
#https://www.pololu.com/file/0J1556/UM7%20Datasheet_v1-8_30.07.2018.pdf

#All SPI operations begin when the master writes two control bytes to the bus. The first byte
#indicates whether the operation is a register read (0x00) or a write (0x01). The second byte is
#the address of the register being accessed. 
#first byte read or write, second is address


#pg 15 for talking about packets    This is spi though, Not UART?
#ser.write(0x00)

#pip install

#Wants Gyroscopic accelerometer and magnetometer data
#Data Registers
# XGyro = 0x61  # Processed x-axis rate gyro data
# YGyro = 0x62  # Processed y-axis rate gyro data
# ZGyro = 0x63  # Processed z-axis rate gyro data 
# XAccel = 0x65  # Processed x-axis accel data
# YAccel = 0x66  # Processed y-axis accel data
# ZAccel = 0x67  # Processed z-axis accel data 
# XMag = 0x69  # Processed x-axis magnetometer data
# YMag = 0x6A  # Processed y-axis magnetometer data
# ZMag = 0x6B  # Processed z-axis magnetometer data 

# #https://stackoverflow.com/questions/57982782/type-codes-in-python-for-array this is int
# type = "<b"

# ser = serial.Serial(port="/dev/ttyS4", baudrate=baud)
# ser.close()
# ser.open()

# cmd_buffer = [0,0,0,0,0,0,0,0,0,0,0]
# sending_buffer = [0,0,0,0,0,0,0]

#Sets desired transmission rates for CHR NMEA-style packets (31 bits) Pg. 55
#CREG_COM_RATES7 = 0x07
#packetRates = 0xf0000







# def turnOffRegister(register):
#     cmd_buffer[0] = ord('s') 
#     cmd_buffer[1] = ord('n')
#     cmd_buffer[2] = ord('p')
#     cmd_buffer[3] = 0x80
#     cmd_buffer[4] = register
#     #Data that is non zero
#     cmd_buffer[5] = 0x00
#     cmd_buffer[6] = 0x00
#     cmd_buffer[7] = 0x00
#     cmd_buffer[8] = 0x00
#     checksum = ord('s') + ord('n') + ord('p') + register + 0x80
#     cmd_buffer[9] = checksum >> 8     #0x02
#     cmd_buffer[10] = checksum & 0xff  #0x1C
    
#     ser.write(bytes(cmd_buffer))
   

# def enable_nmea(register):
#     cmd_buffer[0] = ord('s') 
#     cmd_buffer[1] = ord('n')
#     cmd_buffer[2] = ord('p')
#     cmd_buffer[3] = 0x80
#     cmd_buffer[4] = register #Processed Data

#     #Data that is non zero
#     cmd_buffer[5] = 0
#     cmd_buffer[6] = 0x01
#     cmd_buffer[7] = 0
#     cmd_buffer[8] = 0

#     checksum = ord('s') + ord('n') + ord('p') + register + 0x80 + 0x01

#     cmd_buffer[9] = checksum >> 8     #0x02
#     cmd_buffer[10] = checksum & 0xff  #0x1C

#     ser.write(cmd_buffer)
#     print(cmd_buffer)

#     return 0


#checkfirmware()
#resetEKF()
#attempting switch between 3 and 7






#b = ser.read()
#print(b) 
#print(bytes('s', 'utf-8'))
#while ser.isOpen():
    
    #read_data(0x5f)

    # b = ser.read()
    # if (b == bytes('s', 'utf-8')):
    #     b = ser.read()
    #     if (b == bytes('n', 'utf-8')):
    #         b = ser.read()
    #         if (b == bytes('p', 'utf-8')):
    #             break



#print("We got stuff!")
    #Using xGyro gets four bytes of data
    #GyroX = read_data(XGyro)
    #GyroY = read_data(YGyro)
    #GyroZ = read_data(ZGyro)
    #AccelX = read_data(XAccel)
    #AccelY = read_data(YAccel)
    #AccelZ = read_data(ZAccel)
    #MagX = read_data(XMag)
    #MagY = read_data(YMag)
    #MagZ = read_data(ZMag)
    

    

    #print("Gyro:      X: " + GyroX + "Y: " + GyroY + "Z: " + GyroZ + "\n         Accel:     X: " + AccelX + "Y: " + AccelY + "Z: " + AccelZ + "\n         Mag:       X: " + MagX + "Y: " + MagY + "Z: " + MagZ)
#ser.close()
