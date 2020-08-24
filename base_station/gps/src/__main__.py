import serial
import time
from rover_common import aiolcm
from rover_msgs import RTCM

# To be moved to config
_BAUDRATE = 115200
_FILENAME = '/dev/ttyACM0'
_BUFFSIZE = 1024


def main():
    lcm = aiolcm.AsyncLCM()
    rtcm = RTCM()
    with serial.Serial(_FILENAME, _BAUDRATE) as s:
        while (s.is_open):
            try:
                byte_string = s.read(_BUFFSIZE)
                print(byte_string)
            except Exception as e:
                print(e)
                time.sleep(1)
                continue
            rtcm.size = len(byte_string)
            rtcm.data = list(byte_string)
            lcm.publish('/rtcm', rtcm.encode())


if __name__ == "__main__":
    main()
