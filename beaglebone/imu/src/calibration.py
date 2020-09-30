from . import imu
from . import mux
import time as t


def main():
    mux.enable(0x1)

    offsets = [0, 0, 0, 0, 0, 0, 0]

    while not imu.start_up():
        pass

    # averages first 50 samples of the accel and gyro data
    for _ in range(50):
        try:
            data = imu.get_data()

            # could just be a for loop, using constants for readbility
            offsets[imu.XACCEL] += data[imu.XACCEL]
            offsets[imu.YACCEL] += data[imu.YACCEL]

            # in the z direction acceleration should be 1, so the offset is the difference
            # accel units at 2048 LSB/g
            offsets[imu.ZACCEL] += (data[imu.ZACCEL] - 2048)

            offsets[imu.XGYRO] += data[imu.XGYRO]
            offsets[imu.YGYRO] += data[imu.YGYRO]
            offsets[imu.ZGYRO] += data[imu.ZGYRO]

        except Exception:
            print("Connection Lost")
            t.sleep(1)

    for i in range(len(offsets)):
        offsets[i] = offsets[i] / -50

    with open('ag_calibration.txt', 'w') as f:
        f.write("{} {} {} \n".format(offsets[imu.XACCEL], offsets[imu.YACCEL], offsets[imu.ZACCEL]))
        f.write("{} {} {} \n".format(offsets[imu.XGYRO], offsets[imu.YGYRO], offsets[imu.ZGYRO]))


if(__name__ == '__main__'):
    main()
