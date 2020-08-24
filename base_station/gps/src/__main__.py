import socket
import time
from rover_common import aiolcm
from rover_msgs import RTCM

# To be moved to config
_TCP_IP = '127.0.0.1'
_TCP_PORT = 2222
_BUFFER_SIZE = 4096


def main():
    lcm = aiolcm.AsyncLCM()
    rtcm = RTCM()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((_TCP_IP, _TCP_PORT))
        while (s):
            try:
                byte_string = s.recv(_BUFFER_SIZE)
                print(byte_string, len(byte_string))
            except Exception as e:
                print(e)
                time.sleep(1)
                continue
            rtcm.size = len(byte_string)
            rtcm.data = list(byte_string)
            lcm.publish('/rtcm', rtcm.encode())


if __name__ == "__main__":
    main()
