
from rover_common import aiolcm
from rover_msgs import OdometryF
from rover_common.aiohelper import run_coroutines
# from rover_msgs import Filtered_Odom
# from rover_msgs import Wheelenc
# from rover_msgs import IMU

UNDEFINED = 0  # -999
INFREQUENCY = 0.2  # inverse of frequency of slowest sensor (probably GPS)


def received(channel, msg):
    m = OdometryF.decode(msg)
    print('--------')
    print(m.latitude_deg)
    print(m.latitude_min)
    print(m.longitude_deg)
    print(m.longitude_min)
    print(m.bearing_deg)
    print(m.speed)
    print('shit was received')


def main():
    # lcm_ = aiolcm.AsyncLCM()
    lcm_ = aiolcm.AsyncLCM()
    lcm_.subscribe("/odometryf", received)
    run_coroutines(lcm_.loop())


if __name__ == "__main__":
    main()
