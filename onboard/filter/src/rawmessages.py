import time
from rover_msgs import OdometryF  # filtered odometry
UNDEFINED = 0


class clean_odom:
    def __init__(self):
        self.speed = UNDEFINED
        self.latitude_deg = UNDEFINED
        self.latitude_min = UNDEFINED
        self.longitude_deg = UNDEFINED
        self.longitude_min = UNDEFINED
        self.bearing_deg = UNDEFINED
        self.speed = UNDEFINED

    def create_lcm(self):
        msg = OdometryF()
        msg.latitude_deg = self.latitude_deg
        msg.latitude_min = self.latitude_min
        msg.longitude_deg = self.longitude_deg
        msg.longitude_min = self.longitude_min
        msg.bearing_deg = self.bearing_deg
        msg.speed = self.speed
        return msg

    def copy_gps(self, gps):
        self.latitude_deg = gps.lat_deg
        self.latitude_min = gps.lat_min
        self.longitude_deg = gps.long_deg
        self.longitude_min = gps.long_min
        self.bearing_deg = gps.track_theta
        self.speed = gps.ground_speed
        return None


class mag_bearing:
    def __init__(self):
        self.mbearing = UNDEFINED
        self.time_of_IMU = time.clock()

    def update_mag_bearing(self, message):
        self.mbearing = message.bearing
        self.time_of_IMU = time.clock()


class raw_gps:
    def __init__(self):
        self.lat_deg = UNDEFINED
        self.lat_min = UNDEFINED
        self.long_deg = UNDEFINED
        self.long_min = UNDEFINED
        self.track_theta = UNDEFINED
        self.ground_speed = UNDEFINED
        self.time_of_GPS = time.clock()

    def updateGPS(self, message):
        self.lat_deg = message.latitude_deg
        self.lat_min = message.latitude_min
        self.long_deg = message.longitude_deg
        self.long_min = message.longitude_min
        self.track_theta = message.bearing_deg
        self.ground_speed = message.speed
        self.time_of_GPS = time.clock()


class nav_status:
    def __init__(self):
        self.navState = UNDEFINED
        self.time_of_status = time.clock()

    def update_nav_status(self, message):
        self.navState = message.nav_state
        self.time_of_IMU = time.clock()
