import serial
import struct
from rover_common import aiolcm
from rover_msgs import Odometry


lcm_ = aiolcm.AsyncLCM()

START = b'\x12'
END = b'\x13'


class Status():
    WaitHeader = 0
    InMsg = 1


class Reader():

    def __init__(self):
        self.state = Status.WaitHeader
        self.buffer = []

    def feed(self, c):
        next_state = self.state
        if self.state == Status.WaitHeader:
            if c == START:
                next_state = Status.InMsg
                self.buffer = []
        elif self.state == Status.InMsg:
            if c == END:
                next_state = Status.WaitHeader
                self.state = next_state
                return True
            self.buffer.append(c)
        self.state = next_state
        return False


def main():
    ser = serial.Serial('/dev/ttyUSB0', timeout=1)
    ser.baudrate = 115200
    r = Reader()
    while True:
        c = ser.read()
        if r.feed(c):
            try:
                roll, pitch, bearing, lat_deg, lat_min, lon_deg, \
                    lon_min = struct.unpack('<fffffff', b''.join(r.buffer))
                msg = Odometry()
                msg.latitude_deg = int(lat_deg)
                msg.latitude_min = lat_min
                msg.longitude_deg = int(lon_deg)
                msg.longitude_min = lon_min
                msg.bearing_deg = bearing
                lcm_.publish('/odom', msg.encode())
            except struct.error:
                print("Incomplete data")
