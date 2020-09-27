import numpy as np
import time as t
import smbus
# import time as t

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

RAW_DATA_0_RDY_INT = 0x1A


AK09916_I2C_ADDR = 0x0c
bus = smbus.SMBus(2)

XACCEL = 0
YACCEL = 1
ZACCEL = 2

XGYRO = 3
YGYRO = 4
ZGYRO = 5

XMAG = 6
YMAG = 7
ZMAG = 8


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

    # waits for the data to be ready 
    while not (bus.read_byte_data(I2C_IMU_ADDRESS, RAW_DATA_0_RDY_INT) & 0x1):
        print("data not ready")
        t.sleep(0.001)

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


def start_up():
    try:
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
        set_offset(0, 0, 0)
        return True

    except Exception:
        return False


def main():

    offsets = [0, 0, 0, 0, 0, 0, 0]

    while not start_up():
        pass

    # averages first 50 samples of the accel and gyro data
    for _ in range(50):
        try:
            data = get_data()

            # could just be a for loop, using constants for readbility
            offsets[XACCEL] += data[XACCEL]
            offsets[YACCEL] += data[YACCEL]

            # in the z direction acceleration should be 1, so the offset is the difference
            # accel units at 2048 LSB/g 
            offsets[ZACCEL] += (data[ZACCEL] - 2048)

            offsets[XGYRO] += data[XGYRO]
            offsets[YGYRO] += data[YGYRO]
            offsets[ZGYRO] += data[ZGYRO]

        except Exception:
            print("Connection Lost")
            t.sleep(1)

    for i in range(len(offsets)):
        offsets[i] = offsets[i] / -50

    with open('ag_calibration.txt', 'w') as f:
        f.write("{} {} {} \n".format(offsets[XACCEL], offsets[YACCEL], offsets[ZACCEL]))
        f.write("{} {} {} \n".format(offsets[XGYRO], offsets[YGYRO], offsets[ZGYRO]))


if(__name__ == '__main__'):
    main()
