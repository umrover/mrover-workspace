import serial
import time
from rover_common import aiolcm
from rover_msgs import RTCM

# To be moved to config
_BAUDRATE = 115200
_FILENAME = '/dev/ttyACM0'


def main():
    lcm = aiolcm.AsyncLCM()
    rtcm = RTCM()
    with serial.Serial(_FILENAME, _BAUDRATE) as s:
        while (s.is_open):
            try:
                msg = str(s.readline())
                print(msg)
            except Exception as e:
                print(e)
                time.sleep(1)
                continue
            rtcm.msg = msg
            lcm.publish('/rtcm', rtcm.encode())


if __name__ == "__main__":
    main()
