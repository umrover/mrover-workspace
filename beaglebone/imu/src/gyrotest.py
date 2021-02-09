# import numpy as np
# import lcm
# import smbus
# # import time as t
# from . import madgwickahrs as MadgwickAHRS
# from rover_msgs import IMUData

# I2C_IMU_ADDRESS = 0x69

# ICM20948_I2C_SLV0_ADDR = 0x03
# ICM20948_I2C_SLV0_REG = 0x04
# ICM20948_I2C_SLV0_CTRL = 0x05
# ICM20948_I2C_SLV0_DO = 0x06
# ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B
# # Bank 0
# ICM20948_USER_CTRL = 0x03
# ICM20948_PWR_MGMT_1 = 0x06
# ICM20948_PWR_MGMT_2 = 0x07
# ICM20948_INT_PIN_CFG = 0x0F

# AK09916_I2C_ADDR = 0x0c
# bus = smbus.SMBus(2)

# filter = MadgwickAHRS.MadgwickAHRS()


# # Combines data for a accel/gyro reading
# def get_decimal(ls, ms):
#     high = read_data(ms) << 8
#     low = read_data(ls) & 0xff
#     return np.int16((high | low))


# # Magnetometer has a different format so seperate function required
# def get_mag_decimal(ls, ms):
#     # little endian so ls is right 8 bits
#     high = bus.read_byte_data(0x0c, ms) << 8
#     low = bus.read_byte_data(0x0c, ls) & 0xff
#     return np.int16((high | low))


# # Gets data from a certain address
# def read_data(num):
#     a = bus.read_byte_data(I2C_IMU_ADDRESS, num)
#     # print(a)
#     return a


# # Sets up Magnetometer Data
# def read_mag_data(ls, ms):
#     # mag_write(AK09916_CNTL2, 0x01) Set magnetometer to singlemeasurementmode
#     bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_USER_CTRL, 0b00000000)
#     # while (ready != 1): # Wait until data is ready
#     # time.sleep(0.00001)
#     bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_INT_PIN_CFG, 0b00000010)
#     # puts the i2c master into bypass mode
#     bus.write_byte_data(0x0c, 0x32, 0b00010)
#     # 0c holds the address of the magnetometer
#     bus.write_byte_data(0x0c, 0x31, 0b00010)
#     # These are settings for the magnetometer
#     return get_mag_decimal(ls, ms)


# # Changes the bank of commands to access
# def set_bank(bank):
#     newbank = (bank << 4)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x7f, newbank)


# def get_data(xav, yav, zav):
#     set_bank(0)
#     # print(bus.read_byte_data(I2C_IMU_ADDRESS, 0x1A))
#     # if((bus.read_byte_data(I2C_IMU_ADDRESS, 0x1A) & 0b00000001) == 0b00000001):
#     #    print("Data Ready")
#     #    t.sleep(0.1)
#     accel_x = 0
#     accel_y = 0
#     accel_z = 0
#     gyro_x = get_decimal(0x34, 0x33)
#     gyro_y = get_decimal(0x36, 0x35)
#     gyro_z = get_decimal(0x38, 0x37)
#     # No Calibration for Mag as that would be near impossible unless you know magnetometer readings
#     mag_x = 0
#     mag_y = 0
#     mag_z = 0

#     bus.read_byte_data(0x0c, 0x18)
#     return np.array([accel_x, accel_y, accel_z, gyro_x,
#                     gyro_y, gyro_z, mag_x, mag_y, mag_z])


# # Removes any inbuilt offset b/c we do that ourselves
# def set_offset(xav, yav, zav):
#     xav /= 10
#     yav /= 10
#     zav /= -10
#     set_bank(1)
#     zavlower = (int(zav) & 0b000000001111111) << 1
#     zavupper = (int(zav) & 0b111111110000000) >> 7

#     # This function is only ever used to 0 out offsets so its fine if they're the same
#     # Set lower bits of z offset
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x1B, zavlower)
#     # Set upper bits of z offset
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x1A, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x14, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x15, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x17, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x18, zavupper)
#     set_bank(2)
#     # Set lower bits of z offset
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x03, zavlower)
#     # Set upper bits of z offset
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x04, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x05, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x06, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x07, zavupper)
#     bus.write_byte_data(I2C_IMU_ADDRESS, 0x08, zavupper)
#     set_bank(0)


# def main():

#     global lcm_
#     lcm_ = lcm.LCM()

#     success = False

#     imudata = IMUData()

#     """ f = open("calibvalues.txt", "r")  # Calibration done outside/before
#     # Get this file by running calibration.py and moving the file to the main workspace dir
#     # This really only needs to be done once
#     if f.mode == 'r':
#         xav = int(f.readline())
#         yav = int(f.readline())
#         zav = int(f.readline())
#         # xgyr = int(f.readline())
#         # ygyr = int(f.readline())
#         # zgyr = int(f.readline()) """
#     while not success:
#         try:
#             print("Attempting Startup")
#             set_bank(0)
#             bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_2, 0b00000111)
#             # wake up imu from sleep, try until works
#             bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_1, 0x01)
#             # Set accelerometer and gyroscope to on
#             bus.write_byte_data(I2C_IMU_ADDRESS, ICM20948_PWR_MGMT_2, 0x00)
#             set_bank(2)
#             bus.write_byte_data(I2C_IMU_ADDRESS, 0x01, 0b00000110)
#             bus.write_byte_data(0x0c, 0x18, 0b00000)
#             # bus.write_byte_data(I2C_IMU_ADDRESS, 0x14, 0b00000110)
#             set_bank(0)
#             # clear any inbuilt offset, we handle that in our code
#             set_offset(0, 0, 0)
#             success = True

#         except Exception:

#             pass

#     while(True):
#         try:
#             data = get_data(xav, yav, zav)

#         except Exception:
#             print("Connection Lost")

#         # Raw Data
#         # print("Accel: ", data[0]/2048, ",", data[1]/2048, ",", data[2]/2048 - 1)
#         # print("Gyro: ", data[3]/16.4, ",", data[4]/16.4, ",", data[5]/16.4)
#         print("Mag: ", data[6], ",", data[7], ",", data[8])

#         """ # Accel measures in 2048 LSB/g and Gyro in 2000 LSB/dps
#         # so we divide the register value by that to get the unit
#         imudata.accel_x_g = data[0]/2048
#         imudata.accel_y_g = data[1]/2048
#         # This is -1 so calibration doesn't mess with regular gravity
#         imudata.accel_z_g = data[2]/2048 - 1 """
#         imudata.gyro_x_dps = data[3]/16.4
#         imudata.gyro_y_dps = data[4]/16.4
#         imudata.gyro_z_dps = data[5]/16.4
#         # Magnetometer is in 0.15 microTeslas/LSB
#         # so we multiply instead but also divide
#         # by 1,000,000 to go to regular teslas
#         """ imudata.mag_x_T = data[6]*0.15/1000000
#         imudata.mag_y_T = data[7]*0.15/1000000
#         imudata.mag_z_T = data[8]*0.15/1000000 """

#         """ # Bearing Calculation

#         # First turn teslas into gauss = teslas*10,000
#         gaussx = imudata.mag_x_T*10000
#         gaussy = imudata.mag_y_T*10000 """
#         # Only depends on x/y
#         #
#         # if gaussy > 0:
#         #    imudata.bearing_deg = 90 - (np.arctan2(gaussx,
#         #                                           gaussy))*180/3.14159265
#         # elif gaussy < 0:
#         #    imudata.bearing_deg = 270 - (np.arctan2(gaussx,
#         #                                            gaussy))*180/3.14159265
#         # elif gaussy == 0 and gaussx < 0:
#         #    imudata.bearing_deg = 180
#         # elif gaussy == 0 and gaussx > 0:
#         #    imudata.bearing_deg = 0

#         # Easier version of above, measures from about -180 to 180
#         """ imudata.bearing_deg = np.arctan2(gaussx, gaussy) * 180/np.pi """
#         """ # print("Bearing: ", imudata.bearing_deg)

#         # Roll, Pitch, yaw calc
#         # Including gravity in calib (-1) gives different results from not accounting for gravity
#         acc = np.array([data[0]/2048, data[1]/2048, (data[2]/2048) -1])
#         gyr = np.array([data[3]/16.4, data[4]/16.4, data[5]/16.4])
#         mag = np.array([data[6], data[7], data[8]])
#         gyr_rad = gyr * (np.pi/180)
#         # Everything Past here is Auton stuff
#         filter.update(gyr_rad, acc, mag)
#         # aboves update method can be run instead of update_imu
#         # if the magnetometer problem is fixed
#         # filter.update_imu(gyr_rad,acc)
#         # #updates the filter and returns roll,pitch,and yaw in quaternion form
#         ahrs = filter.quaternion.to_euler_angles()

#         # values are between -pi and pi
#         curRoll = ahrs[0]
#         curPitch = ahrs[1]
#         curYaw = ahrs[2]

#         # Remove prints after testing
#         print("Roll: ", curRoll, " Pitch: ", curPitch, " Yaw: ", curYaw)

#         imudata.roll_rad = curRoll
#         imudata.pitch_rad = curPitch
#         imudata.yaw_rad = curYaw

#         lcm_.publish('/imu_data', imudata.encode()) """


# if(__name__ == '__main__'):
#     main()
