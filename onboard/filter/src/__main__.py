import time
import enum
from rover_common import aiolcm
import asyncio
from rover_msgs import Bearing
from rover_msgs import Odometry
from rover_msgs import NavStatus
from rover_common.aiohelper import run_coroutines
from .rawmessages import mag_bearing
from .rawmessages import raw_gps
from .rawmessages import nav_status
from .rawmessages import clean_odom

# from rover_msgs import Wheelenc
# from rover_msgs import IMU

UNDEFINED = 0
INFREQUENCY = 0.2  # inverse of frequency of slowest sensor (probably GPS)


class NavState(enum.Enum):
    Off = 0
    Done = 1
    Turn = 10
    Drive = 11
    SearchFaceNorth = 20
    SearchFace120 = 21
    SearchFace240 = 22
    SearchFace360 = 23
    SearchTurn = 24
    SearchDrive = 25
    TurnToBall = 28
    DriveToBall = 29
    TurnAroundObs = 30
    DriveAroundObs = 31
    SearchTurnAroundObs = 32
    SearchDriveAroundObs = 33
    Unknown = 255


class FilterClass:
    def __init__(self):
        self.gps = raw_gps()
        self.imu_bearing = mag_bearing()
        self.navstat = nav_status()
        self.odomf = clean_odom()

    def gps_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('gps callback called')
        m = Odometry.decode(msg)
        self.gps.updateGPS(m)
        self.odomf.copy_gps(self.gps)
        print('gps track angle: ' + str(self.gps.track_theta))
        return None

    def mag_bearing_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('imu callback called')
        m = Bearing.decode(msg)
        self.imu_bearing.update_mag_bearing(m)
        print('mag bearing: ' + str(self.imu_bearing.mbearing))
        return None

    def navstat_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('navstat callback called')
        m = NavStatus.decode(msg)
        self.navstat.update_nav_status(m)
        if self.turning():
            print('turning')
        else:
            print("not turning")
        print('nav status: ' + str(self.navstat.navState))
        return None

    def stationary(self):
        """Determine if rover is stationary."""
        if self.navstat == NavState.Off or \
           self.navstat == NavState.Done:
            return True
        return False

    def turning(self):
        """Determine if rover is turning."""
        if self.navstat.navState == NavState.Turn or \
           self.navstat.navState == NavState.SearchTurn or \
           self.navstat.navState == NavState.SearchFaceNorth or \
           self.navstat.navState == NavState.SearchFace120 or \
           self.navstat.navState == NavState.SearchFace240 or \
           self.navstat.navState == NavState.SearchFace360 or \
           self.navstat.navState == NavState.TurnToBall or \
           self.navstat.navState == NavState.TurnAroundObs or \
           self.navstat.navState == NavState.SearchTurnAroundObs:
            return True

        return False

    def driving(self):
        """Determine if rover is driving."""
        if self.navstat.navState == NavState.Drive or \
           self.navstat.navState == NavState.SearchDrive or \
           self.navstat.navState == NavState.DriveToBall or \
           self.navstat.navState == NavState.DriveAroundObs or \
           self.navstat.navState == NavState.SearchDriveAroundObs:
            return True

        return False

    def filter_bearing():
        return None

    # this function is run as a co-routine for publishing fused odometry
    async def publishOdom(self, lcm_):
        while True:
            print('async af')
            # self.filter_bearing()
            msg = self.odomf.create_lcm()
            lcm_.publish('/odometryf', msg.encode())
            await asyncio.sleep(1)

        return None


def main():
    lcm_ = aiolcm.AsyncLCM()
    filter_ = FilterClass()

    lcm_.subscribe("/odometry", filter_.gps_callback)
    lcm_.subscribe("/bearing", filter_.mag_bearing_callback)
    lcm_.subscribe("/nav_status", filter_.navstat_callback)

    run_coroutines(lcm_.loop(), filter_.publishOdom(lcm_))


if __name__ == "__main__":
    main()
