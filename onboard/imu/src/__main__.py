import os
from serial import Serial
from configparser import ConfigParser
from rover_common import aiolcm
from rover_msgs import Bearing


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
        serial_port.read_until(b',')  # angle_x
        serial_port.read_until(b',')  # angle_y
        angle_z = float(serial_port.read_until(b'\r'))

        if num_readings == 0:
            calibrate_start = angle_z
        elif num_readings == calibration_readings:
            calibrate_end = angle_z
            drift = (calibrate_end - calibrate_start) / calibration_readings
            calibrated = True
            print('Done.')

        num_readings += 1

        if calibrated:
            bearing_msg = Bearing()
            bearing_msg.bearing = (angle_z - (drift * num_readings)) % 360
            lcm.publish('/bearing', bearing_msg.encode())


if __name__ == "__main__":
    main()
