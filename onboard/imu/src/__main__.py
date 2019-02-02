import os
from serial import Serial
from configparser import ConfigParser
from rover_common import aiolcm
from rover_msgs import IMU


def read_value(serial_port, delimiter=b','):
    return float(serial_port.read_until(delimiter)[:-1])


def main():
    lcm = aiolcm.AsyncLCM()

    settings_path = os.environ['MROVER_CONFIG'] + '/config_imu/config.ini'

    config = ConfigParser()
    config.read(settings_path)

    calibration_readings = int(config['calibration']['readings'])
    serial_port = Serial(config['serial']['port'])
    serial_port.baudrate = int(config['serial']['baud'])

    calibrated = False
    calibrate_start = 0
    calibrate_end = 0
    drift = 0
    num_readings = 0

    print('Calibrating...')

    while (True):
        accel_x = read_value(serial_port)
        accel_y = read_value(serial_port)
        accel_z = read_value(serial_port)

        gyro_x = read_value(serial_port)
        gyro_y = read_value(serial_port)
        gyro_z = read_value(serial_port)

        mag_x = read_value(serial_port)
        mag_y = read_value(serial_port)
        mag_z = read_value(serial_port)

        read_value(serial_port)  # angle_x
        read_value(serial_port)  # angle_y
        angle_z = read_value(serial_port, b'\r')

        if num_readings == 0:
            calibrate_start = angle_z
        elif num_readings == calibration_readings:
            calibrate_end = angle_z
            drift = (calibrate_end - calibrate_start) / calibration_readings
            calibrated = True
            print('Done.')

        num_readings += 1

        if calibrated:
            imu_msg = IMU()
            imu_msg.accel_x = accel_x
            imu_msg.accel_y = accel_y
            imu_msg.accel_z = accel_z
            imu_msg.gyro_x = gyro_x
            imu_msg.gyro_y = gyro_y
            imu_msg.gyro_z = gyro_z
            imu_msg.mag_x = mag_x
            imu_msg.mag_y = mag_y
            imu_msg.mag_z = mag_z
            imu_msg.bearing = (angle_z - (drift * num_readings)) % 360
            lcm.publish('/imu', imu_msg.encode())


if __name__ == "__main__":
    main()
