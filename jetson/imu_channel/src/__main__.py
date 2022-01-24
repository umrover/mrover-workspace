# accel_x accel_y accel_z gyro_x gyro_y gyro_z bearing
import lcm
import serial
from rover_msgs import IMUData
import math


def main():
    lcm_ = lcm.LCM()

    while (1):
        imu_data = IMUData()
        imu_data.mag_x_uT = 0
        imu_data.mag_y_uT = 0
        imu_data.mag_z_uT = 0
        imu_data.roll_rad = 0
        imu_data.pitch_rad = 0
        imu_data.yaw_rad = 0

        with serial.Serial('/dev/ttyS1', 115200, timeout=1) as ser:
            line = ser.readline()
            vals = [float(val.strip()) for val in line.split()]
            imu_data.accel_x_g = vals[0]
            imu_data.accel_y_g = vals[1]
            imu_data.accel_z_g = vals[2]

            imu_data.gyro_x_dps = vals[3] * (180.0/math.pi)
            imu_data.gyro_y_dps = vals[4] * (180.0/math.pi)
            imu_data.gyro_z_dps = vals[5] * (180.0/math.pi)

            imu_data.bearing_deg = vals[6]

            lcm_.publish("/imu_data", imu_data.encode())


if __name__ == "__main__":
    main()
