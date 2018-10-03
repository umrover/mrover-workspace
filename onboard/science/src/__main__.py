import serial
import struct
import time
import os
from rover_common import aiolcm, frame_serial
from rover_msgs import Sensors


lcm_ = aiolcm.AsyncLCM()
sensors_port = os.environ.get('MROVER_SENSORS_PORT', '/dev/ttyUSB0')


class ScienceFrame:
    def __init__(self, buf):
        data = struct.unpack('<ffffff', buf)
        self.O2 = data[0]
        self.CO2 = data[1]
        self.moisture = data[2]
        self.pH = data[3]
        self.temperature = data[4]
        self.conductivity = data[5]


def main():
    ser = serial.Serial(sensors_port, timeout=1)
    ser.baudrate = 115200
    r = frame_serial.Reader()
    while True:
        c = ser.read()
        if r.feed(c):
            try:
                frame = ScienceFrame(b''.join(r.buffer))
                msg = Sensors()
                msg.timestamp = int(time.time())
                msg.temperature = frame.temperature
                msg.moisture = frame.moisture
                msg.conductivity = frame.conductivity
                msg.pH = frame.pH
                msg.O2 = frame.O2
                msg.CO2 = frame.CO2
                lcm_.publish('/sensors', msg.encode())
            except struct.error as e:
                print(e)
                # print("Incomplete data, only {} bytes".format(len(r.buffer)))
