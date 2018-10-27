import serial
import math
import time
from rover_common import aiolcm
from rover_msgs import Bearing

calibration_time = 5


def main():
    global calibration_time
    global mag_weight

    lcm = aiolcm.AsyncLCM()

    serial_port = serial.Serial("/dev/ttyACM0")
    serial_port.baudrate = 115200
    start_time = time.time()
    calibrated = False
    initial_mag = 0
    mag_calibrate_sum = 0
    mag_calibrate_num = 0

    print('Calibrating...')

    while (True):
        c = serial_port.read()
        if c != b'$':
            continue

        serial_port.read_until(b',')[:-1]  # accel_x
        serial_port.read_until(b',')[:-1]  # accel_y
        serial_port.read_until(b',')[:-1]  # accel_z
        mag_x = float(serial_port.read_until(b',')[:-1])
        mag_y = float(serial_port.read_until(b',')[:-1])
        serial_port.read_until(b',')[:-1]  # mag_z
        serial_port.read_until(b',')[:-1]  # gyro_x
        serial_port.read_until(b',')[:-1]  # gyro_y
        serial_port.read_until(b'\r')[:-1]  # gyro_z

        mag_bearing = ((math.atan2(mag_y, mag_x) *
                        180 / math.pi) - initial_mag) % 360

        if not calibrated:
            mag_calibrate_sum += mag_bearing
            mag_calibrate_num += 1
            current_time = time.time()
            if current_time - start_time > calibration_time:
                calibrated = True
                initial_mag = mag_calibrate_sum / mag_calibrate_num
                print('Done.')
            else:
                continue

        bearing_msg = Bearing()
        bearing_msg.bearing = mag_bearing
        lcm.publish('/bearing', bearing_msg.encode())


if __name__ == "__main__":
    main()
