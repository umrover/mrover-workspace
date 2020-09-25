import smbus
import time as t
import numpy as np

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


def set_offset(xav, yav, zav):
    xav /= 10
    yav /= 10
    zav /= -10
    set_bank(1)
    zavlower = (int(zav) & 0b000000001111111) << 1
    zavupper = (int(zav) & 0b111111110000000) >> 7
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
    success = False
    calibration = 0
    calibrationtime = 0
    xav = 0
    yav = 0
    zav = 0
    xgyr = 0
    ygyr = 0
    zgyr = 0
    while not success:
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
            success = True
        except Exception:
            t.sleep(1)
            pass

    while(True):
        try:
            data = get_data()
            if (calibrationtime < 50):
                xav += data[0]
                yav += data[1]
                zav += data[2]
                xgyr += data[3]
                ygyr += data[4]
                zgyr += data[5]
                calibrationtime += 1

        except Exception:
            print("Connection Lost")
            t.sleep(1)
        if (calibration == 0 and calibrationtime >= 50):
            xav /= -10
            yav /= -10
            zav /= -10
            xgyr /= -10
            ygyr /= -10
            zgyr /= -10
            calibration = 1
            print(xav)
            print(yav)
            print(zav)
            break
    f = open("accgyrocalib.txt.txt", "w+")
    f.write("%d\n" % xav)
    f.write("%d\n" % yav)
    f.write("%d\n" % zav)
    f.write("%d\n" % xgyr)
    f.write("%d\n" % ygyr)
    f.write("%d\n" % zgyr)
    f.close


if(__name__ == '__main__'):
    main()
