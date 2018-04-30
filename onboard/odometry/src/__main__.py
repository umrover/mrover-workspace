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


class OdomFrame:
    def __init__(self, buf):
        data = struct.unpack('<fffIfIf?', buf)
        self.roll = data[0]
        self.pitch = data[1]
        self.bearing = data[2]
        self.lat_deg = data[3]
        self.lat_min = data[4]
        self.lon_deg = data[5]
        self.lon_min = data[6]
        self.gps_read = data[7]


def main():
    ser = serial.Serial('/dev/ttyUSB0', timeout=1)
    ser.baudrate = 115200
    r = Reader()
    counter = 0
    while True:
        c = ser.read()
        if r.feed(c):
            try:
                frame = OdomFrame(b''.join(r.buffer))
                if not frame.gps_read:
                    print('{}: Could not read a complete GPS frame'.format(
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
