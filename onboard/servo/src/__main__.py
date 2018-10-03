from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import CameraServos
from enum import Enum
import serial
import os

lcm_ = aiolcm.AsyncLCM()
ssc32_port = os.environ.get('MROVER_SSC32_PORT', '/dev/ttyUSB0')
ser = serial.Serial(ssc32_port, 9600, timeout=1)


class Servos(Enum):
    CAMERA_PAN = 1
    CAMERA_TILT = 0


def to_servo_value(amount):
    return 1500 + (amount * 500)


def camera_servo_callback(channel, msg):
    print("Recieving {} bytes from {}".format(len(msg), channel))
    m = CameraServos.decode(msg)
    pan = to_servo_value(m.pan)
    tilt = to_servo_value(m.tilt)
    sendMsg = "#{}P{} #{}P{}".format(Servos.CAMERA_PAN.value, pan,
                                     Servos.CAMERA_TILT.value, tilt)
    sendMsg += "T10\r"
    print(sendMsg)
    ser.write(sendMsg.encode("utf-8"))


def main():
    lcm_.subscribe("/camera_servos", camera_servo_callback)
    run_coroutines(lcm_.loop())


if __name__ == "__main__":
    main()


'''
For serial messages

# <ch> P <pw> S <spd> ... # <ch> P <pw> S <spd> T <time> <cr>
<ch> = Channel number in decimal,0- 31.
<pw> = Pulse width in microseconds, 500 - 2500.
<spd> = Movement speed in uS per second for one channel. (Optional)
<time> = Time in mS for the entire move, affects all channels, 65535 max.
<cr> = Carriage return character, ASCII 13. (Required to initiate action)
<esc> = Cancel the current command, ASCII 27.
Servo Move Example: "#5 P1600 S750 <cr>"
'''
