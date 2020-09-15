import numpy as np
import lcm
import time as t
import smbus
# import time as t
from . import madgwickahrs as MadgwickAHRS
from rover_msgs import IMUData

I2C_IMU_ADDRESS = 0x69

ICM20948_I2C_SLV0_ADDR = 0x03
ICM20948_I2C_SLV0_REG = 0x04
ICM20948_I2C_SLV0_CTRL = 0x05
ICM20948_I2C_SLV0_DO = 0x06
ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B
# Bank 0
ICM20948_USER_CTRL = 0x03
ICM20948_PWR_MGMT_1 = 0x06
ICM20948_PWR_MGMT_2 = 0x07
ICM20948_INT_PIN_CFG = 0x0F

AK09916_I2C_ADDR = 0x0c
bus = smbus.SMBus(2)

# Offsets applied to raw x/y/z mag values
mag_offsets = [0, 0, 0]

# Soft iron error compensation matrix
mag_softiron_matrix = [[0, 0, 0],
                       [0, 0, 0],
                       [0, 0, 0]]

mag_field_strength = 0

filter = MadgwickAHRS.MadgwickAHRS()


def get_decimal(high, low):
    high = high << 8
    low = low & 0xff
    return np.int16((high | low))


# Combines data for a accel/gyro reading
def get_accelgyro_data(addr):
    block = bus.read_i2c_block_data(I2C_IMU_ADDRESS, addr, 12)

    x_accel = get_decimal(block[0], block[1])
    y_accel = get_decimal(block[2], block[3])
    z_accel = get_decimal(block[4], block[5])

    x_gyro = get_decimal(block[6], block[7])
    y_gyro = get_decimal(block[8], block[9])
    z_gyro = get_decimal(block[10], block[11])

    return np.array([x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro])


# Sets up Magnetometer Data
def get_mag_data(addr):
    # mag_write(AK09916_CNTL2, 0x01) Set magnetometer to singlemeasurementmode
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_USER_CTRL, 0b00000000)
    # while (ready != 1): # Wait until data is ready
    # time.sleep(0.00001)
    bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_INT_PIN_CFG, 0b00000010)
    # puts the i2c master into bypass mod
    # 0c holds the address of the magnetometer
    bus.write_byte_data(AK09916_I2C_ADDR, 0x31, 0b00000010)
    # These are settings for the magnetometer

    block = bus.read_i2c_block_data(AK09916_I2C_ADDR, addr, 6)

    x_mag = get_decimal(block[1], block[0])
    y_mag = get_decimal(block[3], block[2])
    z_mag = get_decimal(block[5], block[4])

    # Check for Measure overflow and wait until its updated
    while (bus.read_byte_data(AK09916_I2C_ADDR, 0x18) & 0b00001000 == 0b00001000):
        t.sleep(0.0001)
    return np.array([x_mag, y_mag, z_mag])


# Changes the bank of commands to access
def set_bank(bank):
    newbank = (bank << 4)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x7f, newbank)


def get_data():
    set_bank(0)
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = get_accelgyro_data(0x2d)
    mag_x, mag_y, mag_z = get_mag_data(0x11)

    # not sure why this is here
    bus.read_byte_data(AK09916_I2C_ADDR, 0x18)

    return np.array([accel_x, accel_y, accel_z, gyro_x,
                    gyro_y, gyro_z, mag_x, mag_y, mag_z])


# Removes any inbuilt offset b/c we do that ourselves
# Don't believe we need this after figuring out the calibration
def set_offset(xav, yav, zav):
    xav /= 10
    yav /= 10
    zav /= -10
    set_bank(1)
    zavlower = (int(zav) & 0b000000001111111) << 1
    zavupper = (int(zav) & 0b111111110000000) >> 7

    # This function is only ever used to 0 out offsets so its fine if they're the same
    # Set lower bits of z offset
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x1B, zavlower)
    # Set upper bits of z offset
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x1A, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x14, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x15, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x17, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x18, zavupper)
    set_bank(2)
    # Set lower bits of z offset
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x03, zavlower)
    # Set upper bits of z offset
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x04, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x05, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x06, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x07, zavupper)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x08, zavupper)
    set_bank(0)


def main():

    global lcm_
    lcm_ = lcm.LCM()

    success = False

    imudata = IMUData()

    while not success:
        try:
            print("Attempting Startup")
            set_bank(0)
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_2, 0x7f)
            # wake up imu from sleep, try until works
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_1, 0x01)
            # Set accelerometer and gyroscope to on
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_2, 0x00)
            set_bank(2)
            bus.write_byte_data(I2C_IMU_ADDRESS, 0x01, 0b00000110)
            bus.write_byte_data(I2C_IMU_ADDRESS, 0x14, 0b00000110)
            set_bank(0)
            # clear any inbuilt offset, we handle that in our code
            # set_offset(0, 0, 0)
            success = True

        except Exception:
            pass

    t.sleep(1)

    with open('calibration.txt', 'r') as calibration:
        lines = calibration.readlines()
        mag_offsets = [float(x) for x in lines[0].split()]
        mag_softiron_matrix = np.reshape([float(x) for x in lines[1].split()], (3, 3))
        # mag_field_strength = [float(x) for x in lines[3].split()][0]

    while(True):
        try:
            data = get_data()

        except Exception:
            print("Connection Lost")

        # Raw Data
        print("Accel Raw: ", data[0] / 2048, ",", data[1] / 2048, ",", data[2] / 2048)
        print("Gyro Raw: ", data[3] / 16.4, ",", data[4] / 16.4, ",", data[5] / 16.4)
        print("Mag Raw: ", data[6] * 0.15, ",", data[7] * 0.15, ",", data[8] * 0.15)

        # Accel measures in 2048 LSB/g and Gyro in 2000 LSB/dps
        # so we divide the register value by that to get the unit
        imudata.accel_x_g = data[0] / 2048
        imudata.accel_y_g = data[1] / 2048
        imudata.accel_z_g = data[2] / 2048

        imudata.gyro_x_dps = data[3] / 16.4
        imudata.gyro_y_dps = data[4] / 16.4
        imudata.gyro_z_dps = data[5] / 16.4

        # Magnetometer is in 0.15 microTeslas/LSB
        mag_x = (data[6] * 0.15) - mag_offsets[0]
        mag_y = (data[7] * 0.15) - mag_offsets[1]
        mag_z = (data[8] * 0.15) - mag_offsets[2]

        # Apply mag soft iron error compensation
        imudata.mag_x_uT = mag_x * mag_softiron_matrix[0][0] + mag_y * mag_softiron_matrix[0][1] + \
            mag_z * mag_softiron_matrix[0][2]
        imudata.mag_y_uT = mag_x * mag_softiron_matrix[1][0] + mag_y * mag_softiron_matrix[1][1] + \
            mag_z * mag_softiron_matrix[1][2]
        imudata.mag_z_uT = mag_x * mag_softiron_matrix[2][0] + mag_y * mag_softiron_matrix[2][1] + \
            mag_z * mag_softiron_matrix[2][2]

        # Bearing Calculation

        imudata.bearing_deg = -(np.arctan2(imudata.mag_y_uT, imudata.mag_x_uT) * (180.0 / np.pi))
        if imudata.bearing_deg < 0:
            imudata.bearing_deg += 360
        print("Bearing: ", imudata.bearing_deg)

        # Roll, Pitch, yaw calc
        # Dividing the mag data by 100000 to get units to be Teslas
        acc = np.array([imudata.accel_x_g, imudata.accel_y_g, imudata.accel_z_g])
        gyr = np.array([imudata.gyro_x_dps, imudata.gyro_y_dps, imudata.gyro_z_dps])
        mag = np.array([imudata.mag_x_uT / 1000000.0, imudata.mag_y_uT / 1000000.0,
                        imudata.mag_z_uT / 1000000.0])
        gyr_rad = gyr * (np.pi/180)

        # Everything Past here is Auton stuff
        filter.update(gyr_rad, acc, mag)
        # aboves update method can be run instead of update_imu
        # if the magnetometer problem is fixed
        # filter.update_imu(gyr_rad,acc)
        # updates the filter and returns roll,pitch,and yaw in quaternion form
        ahrs = filter.quaternion.to_euler_angles()

        # values are between -pi and pi
        curRoll = ahrs[0]
        curPitch = ahrs[2]
        curYaw = ahrs[1]

        # Remove prints after testing
        print("Roll: ", curRoll, " Pitch: ", curPitch, " Yaw: ", curYaw)

        imudata.roll_rad = curRoll
        imudata.pitch_rad = curPitch
        imudata.yaw_rad = curYaw

        lcm_.publish('/imu_data', imudata.encode())


if(__name__ == '__main__'):
    main()
