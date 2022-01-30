import lcm
from rover_msgs import MosfetCmd
import time
import os

lcm_ = lcm.LCM()


def main():

    mosfet = MosfetCmd()
    print(os.environ['LCM_DEFAULT_URL'])
    while(True):
        mosfet.device = 5
        mosfet.enable = 1
        lcm_.publish('/mosfet_cmd', mosfet.encode())
        time.sleep(0.5)


if(__name__ == "__main__"):
    main()
