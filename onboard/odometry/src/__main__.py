import serial
import struct
import os
from rover_common import aiolcm, frame_serial
from rover_msgs import Odometry


lcm_ = aiolcm.AsyncLCM()
odom_port = os.environ.get('MROVER_ODOM_PORT', '/dev/ttyUSB0')


class OdomFrame:
    def __init__(self, buf):
        data = struct.unpack('<fffifif??', buf)
        self.roll = data[0]
        self.pitch = data[1]
        self.bearing = data[2]
        self.lat_deg = data[3]
        self.lat_min = data[4]
        self.lon_deg = data[5]
        self.lon_min = data[6]
        self.gps_read = data[7]
        self.imu_read = data[8]


def main():
    ser = serial.Serial(odom_port, timeout=1)
    ser.baudrate = 115200
    r = frame_serial.Reader()
    counter = 0
    while True:
        c = ser.read()
        if r.feed(c):
            try:
                frame = OdomFrame(b''.join(r.buffer))
                if not frame.gps_read:
                    print('{}: Could not read a complete GPS frame'.format(
                        counter))
                if not frame.imu_read:
                    print('{}: Could not read a complete IMU frame'.format(
                        counter))
                counter += 1
                msg = Odometry()
                msg.latitude_deg = int(frame.lat_deg)
                msg.latitude_min = frame.lat_min
                msg.longitude_deg = int(frame.lon_deg)
                msg.longitude_min = frame.lon_min
                msg.bearing_deg = frame.bearing
                lcm_.publish('/odom', msg.encode())
            except struct.error:
                print("Incomplete data, only {} bytes".format(len(r.buffer)))
