import smbus
import time as t
import numpy as np

I2C_IMU_ADDRESS = 0x69
I2C_ADDR = 0x68
I2C_ADDR_ALT = 0x69

ICM20948_I2C_MST_CTRL = 0x01
ICM20948_I2C_MST_DELAY_CTRL = 0x02
# Bank 0


ICM20948_PWR_MGMT_1 = 0x06
ICM20948_PWR_MGMT_2 = 0x07


AK09916_CNTL3 = 0x32

bus = smbus.SMBus(2)


def get_decimal(ls, ms):
    high = read_data(ms) << 8
    low = read_data(ls) & 0xff
    return np.int16((high | low))


def get_mag_decimal(ls, ms):
    # little endian so ls is right 8 bits
    high = bus.read_byte_data(0x0c, ms) << 8
    low = bus.read_byte_data(0x0c, ls) & 0xff
    return np.int16((high | low))


def read_data(num):
    a = bus.read_byte_data(I2C_IMU_ADDRESS, num)
    # print(a)
    return a


def set_bank(bank):
    newbank = (bank << 4)
    bus.write_byte_data(I2C_IMU_ADDRESS, 0x7f, newbank)


def get_data():

    accel_x = get_decimal(0x2E, 0x2D)
    accel_y = get_decimal(0x30, 0x2F)
    accel_z = get_decimal(0x32, 0x31)
    # print("accel x: ", accel_x,"accel y: ",accel_y,"accel z: ",accel_z)
    gyro_x = get_decimal(0x34, 0x33)
    gyro_y = get_decimal(0x36, 0x35)
    gyro_z = get_decimal(0x38, 0x37)

    # all magnetometer data returns 0 because it is not being read correctly

#     print("mag x: ", mag_x,"mag y: ",mag_y,"mag z: ",mag_z)
    return np.array([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z])


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
            print("Attempting Startup")
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_1, 0x01)
            # wake up imu from sleep, try until works
            bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_2, 0x00)
            # Set accelerometer and gyroscope to on
            set_bank(2)
            bus.write_byte_data(I2C_IMU_ADDRESS, 0x01, 0b00000110)
            bus.write_byte_data(I2C_IMU_ADDRESS, 0x14, 0b00000110)
            set_bank(0)
            set_offset(0, 0, 0)  # Clear any offset, we just want the raw data
            t.sleep(1)
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
        if (calibration == 0 and calibrationtime >= 10):
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
    f = open("calibvalues.txt", "w+")
    f.write("%d\n" % xav)
    f.write("%d\n" % yav)
    f.write("%d\n" % zav)
    f.write("%d\n" % xgyr)
    f.write("%d\n" % ygyr)
    f.write("%d\n" % zgyr)
    f.close


if(__name__ == '__main__'):
    main()
