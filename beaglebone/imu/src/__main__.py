import numpy as np
import lcm
from . import madgwickahrs as MadgwickAHRS
from rover_msgs import IMUData

from . import calibration as imu

# Offsets applied to raw x/y/z mag values
gyro_offsets = [0, 0, 0]
accel_offsets = [0, 0, 0]
mag_offsets = [0, 0, 0]

# Soft iron error compensation matrix
mag_softiron_matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

mag_field_strength = 0

filter = MadgwickAHRS.MadgwickAHRS()


def main():
    global lcm_
    lcm_ = lcm.LCM()
    imudata = IMUData()

    while not imu.start_up():
            pass

    with open('mag_calibration.txt', 'r') as mag_calibration:
        lines = mag_calibration.readlines()
        mag_offsets = [float(x) for x in lines[0].split()]
        mag_softiron_matrix = np.reshape([float(x) for x in lines[1].split()], (3, 3))
        # mag_field_strength = [float(x) for x in lines[3].split()][0]

    with open('ag_calibration.txt', 'r') as ag_calibration:
        lines = ag_calibration.readlines()

        accel_offsets = [float(x) for x in lines[0].split()]
        gyro_offsets = [float(x) for x in lines[1].split()]

    while(True):
        try:
            data = imu.get_data()

        except Exception:
            print("Connection Lost")

        # Apply accel/gyro calibration
        for i in range(6):
            if (i < 3):
                data[i] += accel_offsets[i]
            else:
                data[i] += gyro_offsets[i - 3]

        # Raw Data
        print("Accel Raw: ", data[imu.XACCEL] / 2048, ",", data[imu.YACCEL] / 2048, ",", data[imu.ZACCEL] / 2048)
        print("Gyro Raw: ", data[imu.XGYRO] / 16.4, ",", data[imu.YGYRO] / 16.4, ",", data[imu.ZGYRO] / 16.4)
        print("Mag Raw: ", data[imu.XMAG] * 0.15, ",", data[imu.YMAG] * 0.15, ",", data[imu.ZMAG] * 0.15)

        # Accel measures in 2048 LSB/g and Gyro in 2000 LSB/dps
        # so we divide the register value by that to get the unit
        imudata.accel_x_g = data[imu.XACCEL] / 2048
        imudata.accel_y_g = data[imu.YACCEL] / 2048
        imudata.accel_z_g = data[imu.ZACCEL] / 2048

        imudata.gyro_x_dps = data[imu.XGYRO] / 16.4
        imudata.gyro_y_dps = data[imu.YGYRO] / 16.4
        imudata.gyro_z_dps = data[imu.ZGYRO] / 16.4

        # Magnetometer is in 0.15 microTeslas/LSB
        mag_x = (data[imu.XMAG] * 0.15) - mag_offsets[0]
        mag_y = (data[imu.YMAG] * 0.15) - mag_offsets[1]
        mag_z = (data[imu.ZMAG] * 0.15) - mag_offsets[2]

        # Apply mag soft iron error compensation
        imudata.mag_x_uT = mag_x * mag_softiron_matrix[0][0] + \
            mag_y * mag_softiron_matrix[0][1] + mag_z * mag_softiron_matrix[0][2]

        imudata.mag_y_uT = mag_x * mag_softiron_matrix[1][0] + \
            mag_y * mag_softiron_matrix[1][1] + mag_z * mag_softiron_matrix[1][2]

        imudata.mag_z_uT = mag_x * mag_softiron_matrix[2][0] + \
            mag_y * mag_softiron_matrix[2][1] + mag_z * mag_softiron_matrix[2][2]

        # Bearing Calculation
        bearing = -(np.arctan2(imudata.mag_y_uT, imudata.mag_x_uT) * (180.0 / np.pi))
        if (bearing < 0):
            bearing += 360

        imudata.bearing_deg = bearing
        print("Bearing: ", imudata.bearing_deg)

        # Roll, Pitch, yaw calc
        # Dividing the mag data by 100000 to get units to be Teslas
        acc = np.array([imudata.accel_x_g, imudata.accel_y_g, imudata.accel_z_g])
        gyr = np.array([imudata.gyro_x_dps, imudata.gyro_y_dps, imudata.gyro_z_dps])
        mag = np.array([imudata.mag_x_uT / 1000000.0, imudata.mag_y_uT / 1000000.0, imudata.mag_z_uT / 1000000.0])
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
