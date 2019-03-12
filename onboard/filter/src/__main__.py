import time
import enum
from rover_common import aiolcm
import asyncio
from rover_msgs import IMU, GPS, NavStatus, Odometry
from rover_common.aiohelper import run_coroutines
from .rawmessages import raw_imu, raw_gps, nav_status

# from rover_msgs import Wheelenc

UNDEFINED = 0
INFREQUENCY = 0.2  # inverse of frequency of slowest sensor (probably GPS)


class NavState(enum.Enum):
    Off = 0
    Done = 1
    Turn = 10
    Drive = 11
    SearchFaceNorth = 20
    SearchSpin = 21
    SearchSpinWait = 22
    SearchTurn = 24
    SearchDrive = 25
    ChangeSearchAlg = 27
    TurnToBall = 28
    DriveToBall = 29
    TurnAroundObs = 30
    DriveAroundObs = 31
    SearchTurnAroundObs = 32
    SearchDriveAroundObs = 33
    Unknown = 255


class Odom:
    def __init__(self):
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None
        self._bearing = None


class FilterClass:
    def __init__(self):
        self._gps = raw_gps()
        self._imu = raw_imu()
        self._odom = Odom()
        self._navstat = nav_status()

    def gps_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('gps callback called')
        m = GPS.decode(msg)
        self._gps.updateGPS(m)
        print('gps track angle: ' + str(self._gps._track_angle))
        return None

    def imu_bearing_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('imu callback called')
        m = IMU.decode(msg)
        self._imu.update_imu_bearing(m)
        print('mag bearing: ' + str(self._imu._bearing))
        return None

    def navstat_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('navstat callback called')
        m = NavStatus.decode(msg)
        self._navstat.update_nav_status(m)
        if self.turning():
            print('turning')
        else:
            print("not turning")
        print('nav status: ' + str(self._navstat._navState))
        return None

    def stationary(self):
        """Determine if rover is stationary."""
        if self._navstat._navState == NavState.Off or \
           self._navstat._navState == NavState.Done or \
           self._navstat._navState == NavState.SearchSpinWait or \
           self._navstat._navState == NavState.ChangeSearchAlg:
            return True
        return False

    def turning(self):
        """Determine if rover is turning."""
        if self._navstat._navState == NavState.Turn or \
           self._navstat._navState == NavState.SearchTurn or \
           self._navstat._navState == NavState.SearchFaceNorth or \
           self._navstat._navState == NavState.SearchSpin or \
           self._navstat._navState == NavState.TurnToBall or \
           self._navstat._navState == NavState.TurnAroundObs or \
           self._navstat._navState == NavState.SearchTurnAroundObs:
            return True

        return False

    def driving(self):
        """Determine if rover is driving."""
        if self._navstat._navState == NavState.Drive or \
           self._navstat._navState == NavState.SearchDrive or \
           self._navstat._navState == NavState.DriveToBall or \
           self._navstat._navState == NavState.DriveAroundObs or \
           self._navstat._navState == NavState.SearchDriveAroundObs:
            return True

        return False

    def filter_bearing(self):
        self._odom._bearing = self._imu._bearing

    def filter_location(self):
        self._odom._lat_deg = self._gps._lat_deg
        self._odom._lat_min = self._gps._lat_min
        self._odom._long_deg = self._gps._long_deg
        self._odom._long_min = self._gps._long_min

    def create_odom_lcm(self):
        # If some part of Odom is uninitialized, return None
        if self._odom._lat_deg is None or \
           self._odom._lat_min is None or \
           self._odom._long_deg is None or \
           self._odom._long_min is None or \
           self._odom._bearing is None:
            return None

        msg = Odometry()
        msg.latitude_deg = self._odom._lat_deg
        msg.latitude_min = self._odom._lat_min
        msg.longitude_deg = self._odom._long_deg
        msg.longitude_min = self._odom._long_min
        msg.bearing_deg = self._odom._bearing
        msg.speed = -1
        return msg

    # this function is run as a co-routine for publishing fused odometry
    async def publishOdom(self, lcm_):
        while True:
            self.filter_bearing()
            self.filter_location()
            msg = self.create_odom_lcm()
            if msg:
                lcm_.publish('/odometry', msg.encode())
            await asyncio.sleep(0.1)

        return None


def main():
    lcm_ = aiolcm.AsyncLCM()
    filter_ = FilterClass()

    lcm_.subscribe("/gps", filter_.gps_callback)
    lcm_.subscribe("/imu", filter_.imu_bearing_callback)
    lcm_.subscribe("/nav_status", filter_.navstat_callback)

    run_coroutines(lcm_.loop(), filter_.publishOdom(lcm_))


if __name__ == "__main__":
    main()
