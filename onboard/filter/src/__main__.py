import json
import math
# import time
from os import getenv

from rover_common import aiolcm
import asyncio
from rover_msgs import IMU, GPS, NavStatus, Odometry, \
                       SensorPackage
from rover_common.aiohelper import run_coroutines
from .rawmessages import RawIMU, RawGPS, RawSensorPackage, RawNavStatus
from .filterObjects import NavState, Odom, LocationEstimate, \
                           BearingEstimate, Acceleration, Velocity


class SensorFusion:
    """Class for fusing raw sensor data into clean Odometry data."""
    def __init__(self):
        """Initialize Sensor Fusion."""
        self._gps = RawGPS()
        self._imu = RawIMU()
        self._odom = Odom()
        self._sensor_package = RawSensorPackage()
        self._velocity = Velocity(0, 0, 0)
        self._nav_status = RawNavStatus()
        self._bearing_calibrator = 0
        self._gps_started = False
        self._sensor_package_started = False
        self._imu_started = False
        configPath = getenv('MROVER_CONFIG')
        configPath += "/config_filter/config.json"
        with open(configPath, "r") as read_file:
            self.filterConfig = json.load(read_file)

    """
    ================================================================
                        General Functions
    ================================================================
    """

    def gps_callback(self, channel, msg):
        """Process new gps data."""
        # print('gps callback')
        # print('-------------- @time : ' + str(time.clock()))
        new_gps = GPS.decode(msg)
        self._gps.update_gps(new_gps)
        # print('gps track angle: ' + str(self._gps._track_angle))
        self._gps_started = True

    def sensor_package_callback(self, channel, msg):
        """Process new sensor package data."""
        # print('sensor_package callback')
        # print('-------------- @time : ' + str(time.clock()))
        new_sensor_package = SensorPackage.decode(msg)
        self._sensor_package.update(new_sensor_package)
        # bear = self._sensor_package._bearing
        # print('sensor package bearing: {}'.format(bear))
        self._sensor_package_started = True

    def imu_callback(self, channel, msg):
        """Process new imu data."""
        # print('imu callback')
        # print('-------------- @time : ' + str(time.clock()))
        new_imu = IMU.decode(msg)
        self._imu.update_imu(new_imu)
        # print('mag bearing: ' + str(self._imu._bearing))
        self._imu_started = True

    def nav_status_callback(self, channel, msg):
        """Process new nav status data."""
        # print('nav_status callback')
        # print('-------------- @time : ' + str(time.clock()))
        new_nav_status = NavStatus.decode(msg)
        self._nav_status.update_nav_status(new_nav_status)
        # print('nav status: ' + str(self._nav_status._navState))

    def stationary(self):
        """Determine if rover is stationary."""
        if self._nav_status._navState in NavState.StationaryStates:
            return True
        return False

    def rotational(self):
        """Determine if rover is in a rotational state."""
        if self._nav_status._navState in NavState.RotationalStates:
            return True
        return False

    def translational(self):
        """Determine if rover is translational state."""
        if self._nav_status._navState in NavState.TranslationalStates:
            return True
        return False

    def sensors_started(self):
        return self._gps_started and \
               self._sensor_package_started and \
               self._imu_started

    """
    ================================================================
                        Location Sensor Fusion
    ================================================================
    """

    def filter_location(self):
        """
        Filter location estimates to get clean location data and update
        odometry data with this clean location.
        """
        # # Location Estimates
        location_weights = self.filterConfig['filterWeights']['location']
        # Raw GPS
        gps_weight = location_weights['gps']
        gps = LocationEstimate.from_raw_gps(self._gps, gps_weight)
        # Raw Sensor Package
        sp_weight = location_weights['sensorPackage']
        sp = LocationEstimate.from_sensor_package(self._sensor_package,
                                                  sp_weight)
        # Filter1
        filter1_weight = location_weights['filter1']
        filter1 = self.location_filter1()
        if filter1 is not None:
            filter1.update_weight(filter1_weight)

        # average location estimates based on weights
        finalLocation = self.fuse_locations([gps, sp, filter1])

        # set final location estimate
        self._odom.update_location(finalLocation)

    def location_filter1(self):
        """
        Combine  gps data with the velocity added to the old position in a
        weighted average.
        [return] final location estimate of the rover
        """
        old_position = self._odom
        velocity = self.finalize_velocity()
        gps = self._gps

        if velocity is None or velocity.north is None:
            return None

        if old_position._lat_min is None:
            old_position = gps

        if not self.filterConfig["constants"].get("metersToLongMin", None):
            self.set_conversions(gps._lat_deg, gps._lat_min)

        meters_to_longitude_minutes \
            = self.filterConfig["constants"]["metersToLongMin"]
        meters_to_latitude_minutes \
            = self.filterConfig["constants"]["metersToLatMin"]

        lat_degrees = gps._lat_deg
        long_degrees = gps._long_deg
        lat_min = gps._lat_min
        long_min = gps._long_min

        vel_lat_minutes = old_position._lat_min + velocity.north \
            * self.filterConfig["constants"]["imuDeltaTime"] \
            * meters_to_latitude_minutes

        lat_gps = (lat_degrees, lat_min,
                   self.filterConfig["driveWeights"]["GPS"]["loc"])
        lat_vel = (old_position._lat_deg, vel_lat_minutes,
                   self.filterConfig["driveWeights"]["GPS"]["calcVel"])

        vel_long_minutes = old_position._long_min + velocity.north \
            * self.filterConfig["constants"]["imuDeltaTime"] \
            * meters_to_longitude_minutes

        long_gps = (long_degrees, long_min)
        long_vel = (old_position._long_deg, vel_long_minutes)

        gps_estimate = LocationEstimate(lat_gps[0], lat_gps[1],
                                        long_gps[0], long_gps[1], lat_gps[2])
        calc_estimate = LocationEstimate(lat_vel[0], lat_vel[1],
                                         long_vel[0], long_vel[1], lat_vel[2])

        locations = [gps_estimate, calc_estimate]

        return self.fuse_locations(locations)

    """
    ================================================================
                        Location Sensor Fusion Helpers
    ================================================================
    """

    def fuse_locations(self, locations):
        """
        Accept a list of location estimations and fuse them together
        to get the final location estimation.
        [return] the final estimation.
        """
        finalLoc = LocationEstimate()
        locations = self.normalize_location_weights(locations)
        if len(locations) == 0:
            return None

        # Fuse latitudes
        lat_degs = [loc._lat_deg for loc in locations]
        lat_mins = [loc._lat_min for loc in locations]
        finalLoc._lat_deg = min(lat_degs)
        weighted_lat_mins = []
        for i, lat_min in enumerate(lat_mins):
            lat_min += 60 * (lat_degs[i] - finalLoc._lat_deg)
            weighted_lat_mins.append(lat_min * locations[i]._weight)
        finalLoc._lat_min = sum(weighted_lat_mins)

        # Fuse longitudes
        long_degs = [loc._long_deg for loc in locations]
        long_mins = [loc._long_min for loc in locations]
        finalLoc._long_deg = min(long_degs)
        weighted_long_mins = []
        for i, long_min in enumerate(long_mins):
            long_min += 60 * (long_degs[i] - finalLoc._long_deg)
            weighted_long_mins.append(long_min * locations[i]._weight)
        finalLoc._long_min = sum(weighted_long_mins)

        finalLoc.derive_location()
        return finalLoc

    def normalize_location_weights(self, locations):
        """
        Accept a list of location estimations and normalize the weights of
        the valid estimates to 1. If any component is None, that estimation
        will be removed. The remaining estimations will have their weights
        normalized.
        [return] list of normalized remaiaining location estimations.
        """
        validLocations = []
        totalWeight = 0
        for loc in locations:
            if loc is None:
                continue
            loc = loc.check_validity()
            if loc is not None:
                totalWeight += loc._weight
                validLocations.append(loc)
        if totalWeight == 0 and self.sensors_started():
            print('valid location weights totalled 0')
            return []
        for loc in validLocations:
            loc._weight /= totalWeight
        return validLocations

    def generate_absolute_acceleration(self):
        """
        Convert the imu data to absolute coordinite system used by the gps
        and corrects for the pitch angle, also updates the z_acceleration
        [return] absolute_accel object containing
        the acceleration in x (North), y (East), and z directions
        """
        bearing = self._odom._bearing
        pitch = self._imu._pitch
        raw_imu = self._imu

        # Assumption: if acc_x is not None, acc_y and acc_z are not None either
        if bearing is None or pitch is None or raw_imu._acc_x is None:
            return None

        north_acc = raw_imu._acc_x * math.cos(pitch) * math.sin(90 - bearing)
        east_acc = raw_imu._acc_x * math.cos(pitch) * math.cos(90 - bearing)
        z_acc = raw_imu._acc_x * math.sin(pitch)
        return Acceleration(north_acc, east_acc, z_acc)

    def decompose_ground_speed(self):
        """
        Break the ground speed velocity into
        components aligned with gps coordinate system
        [return] vel object containing the velocity
         in the North, East and Z Directions
        """
        ground_speed = self._gps._ground_speed
        bearing = self._odom._bearing

        if ground_speed is None or bearing is None:
            return None

        ground_speed = max(0, ground_speed)
        vel_East = ground_speed * math.cos(90 - bearing)
        vel_North = ground_speed * math.sin(90 - bearing)
        return Velocity(vel_East, vel_North, 0)

    def finalize_velocity(self):
        """
        Combine the ground speed and velocity from
        the imu in a weighted average
        [return] absolute_vel object containing the
        velocity in the North, East, and Z directions
        """
        ground_speed = self.decompose_ground_speed()
        accel = self.generate_absolute_acceleration()
        old_velocity = self._velocity

        if ground_speed is None or accel is None or old_velocity is None:
            return None

        vel_North = self.filterConfig["driveWeights"]["IMU"]["accel"] \
            * (old_velocity.north + accel.north *
                self.filterConfig["constants"]["imuDeltaTime"]) \
            + self.filterConfig["driveWeights"]["GPS"]["groundSpeed"] \
            * ground_speed.north
        vel_East = self.filterConfig["driveWeights"]["IMU"]["accel"] \
            * (old_velocity.east + accel.east *
               self.filterConfig["constants"]["imuDeltaTime"]) \
            + self.filterConfig["driveWeights"]["GPS"]["groundSpeed"] \
            * ground_speed.east
        vel_z = self.filterConfig["driveWeights"]["IMU"]["accel"] \
            * (old_velocity.z + accel.z
               * self.filterConfig["constants"]["imuDeltaTime"])
        self._velocity = Velocity(vel_North, vel_East, vel_z)
        return self._velocity

    def meters_to_long_min(self, lat_deg, lat_min):
        """
        Calculate the conversion factor between meters
        and longitude minutes.
        """
        return 60 / (self.filterConfig["constants"]["earthCirc"] *
                     math.cos(math.radians(lat_deg + lat_min)) / 360)

    def set_conversions(self, lat_deg, lat_min):
        """
        Set the config member for the meters to
        longitude minutes conversion.
        """
        self.filterConfig["constants"]["metersToLongMin"] \
            = self.meters_to_long_min(lat_deg, lat_min)

    """
    ================================================================
                        Bearing Sensor Fusion
    ================================================================
    """

    def filter_bearing(self):
        """
        Filter bearing estimates to get clean bearing data and update
        odometry data with this clean bearing.
        """
        # # Bearing estimates
        bearingWeights = self.filterConfig['filterWeights']['bearing']
        # Raw IMU bearing
        imuWeight = bearingWeights['imu']
        imu = BearingEstimate(self._imu._bearing, imuWeight)
        # IMU yaw calculation
        imuYawWeight = bearingWeights['imuYaw']
        imuYaw = BearingEstimate(self._imu._yaw, imuYawWeight)
        # Raw sensor package
        spWeight = bearingWeights['sensorPackage']
        sp = BearingEstimate(self._sensor_package._bearing, spWeight)
        # track angle and gyro
        trackAngleGyroWeight = bearingWeights['trackAngleGyro']
        trackAngleGyro = self.bearing_filter1()
        if trackAngleGyro is not None:
            trackAngleGyro.update_weight(trackAngleGyroWeight)

        # average bearing estimates based on weights
        finalBearing = self.fuse_bearings([imu, imuYaw, sp, trackAngleGyro])

        # set final location estimate
        self._odom.update_bearing(finalBearing)

    def bearing_filter1(self):
        """Estimate bearing by combining gps track angle and imu gyroscope."""
        if self.translational():
            bearing = self._gps.track_mov_avg()
            if bearing is not None:
                imu_bearing = self._imu.bearing_mov_avg()
                if imu_bearing is not None:
                    self._bearing_calibrator = bearing - imu_bearing
        else:
            bearing = self._imu.bearing_mov_avg()
            if bearing is not None:
                bearing += self._bearing_calibrator
        return BearingEstimate(bearing)

    """
    ================================================================
                        Bearing Sensor Fusion Helpers
    ================================================================
    """

    def fuse_bearings(self, bearings):
        """
        Accept a list of bearing estimations and fuse them together
        to get the final bearing estimation.
        [return] final bearing estimation.
        """
        bearings = self.normalize_bearing_weights(bearings)
        if len(bearings) == 0:
            return None
        min_bearing = min([bearing._bearing for bearing in bearings])
        max_bearing = max([bearing._bearing for bearing in bearings])
        if(max_bearing - min_bearing) > 180:
            bears = []
            for bearing in bearings:
                bear = bearing._bearing
                bears.append(bear if bear < 180 else bear + 360)
                bears[-1] *= bearing._weight
        else:
            bears = [bear._bearing * bear._weight for bear in bearings]
        return BearingEstimate(sum(bears))

    def normalize_bearing_weights(self, bearings):
        """
        Accept a list of bearing estimations and normalize the weights of
        the valid estimates to 1. If any component is None, that estimation
        will be removed. The remaining estimations will have their weights
        normalized.
        [return] list of normalized remaiaining bearing estimations.
        """
        validBearings = []
        totalWeight = 0
        for bearing in bearings:
            if bearing is None:
                continue
            bearing = bearing.check_validity()
            if bearing is not None:
                totalWeight += bearing._weight
                validBearings.append(bearing)
        if totalWeight == 0 and self.sensors_started():
            print('bearing weights totalled 0')
            return []
        for bearing in validBearings:
            bearing._weight /= totalWeight
        return validBearings

    """
    ================================================================
                        Combined Sensor Fusion
    ================================================================
    """

    async def calc_odom(self, lcm_):
        """
        Filter raw bearing and location data and publish final clean
        odometry data.
        """
        while True:
            self.filter_bearing()
            self.filter_location()
            odom = self.create_odom_lcm()
            if odom:
                # print('ODOM MESSAGE:')
                # print('....lat deg', odom.latitude_deg)
                # print('....lat min', odom.latitude_min)
                # print('....long deg', odom.longitude_deg)
                # print('....long min', odom.longitude_min)
                # print('....bearing', odom.bearing_deg)
                # print('....speed', odom.speed)
                lcm_.publish('/odometry', odom.encode())
            await asyncio.sleep(self.filterConfig["constants"]["updateRate"])

        return None  # TODO: do we need this??

    def create_odom_lcm(self):
        """Create LCM message from current clean odometry data."""
        # If some part of Odom is uninitialized, return None
        if self._odom._lat_deg is None or \
           self._odom._lat_min is None or \
           self._odom._long_deg is None or \
           self._odom._long_min is None or \
           self._odom._bearing is None:
            return None

        odom = Odometry()
        odom.latitude_deg = int(self._odom._lat_deg)
        odom.latitude_min = self._odom._lat_min
        odom.longitude_deg = int(self._odom._long_deg)
        odom.longitude_min = self._odom._long_min
        odom.bearing_deg = self._odom._bearing
        odom.speed = -1
        return odom


def main():
    lcm_ = aiolcm.AsyncLCM()
    filter_ = SensorFusion()

    lcm_.subscribe("/gps", filter_.gps_callback)
    lcm_.subscribe("/imu", filter_.imu_callback)
    lcm_.subscribe("/nav_status", filter_.nav_status_callback)
    lcm_.subscribe("/sensor_package", filter_.sensor_package_callback)

    run_coroutines(lcm_.loop(), filter_.calc_odom(lcm_))


if __name__ == "__main__":
    main()
