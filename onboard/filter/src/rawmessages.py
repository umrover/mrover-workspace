import time
from rover_msgs import Odometry  # filtered odometry


class clean_odom:
    def __init__(self):
        self._latitude_deg = None
        self._latitude_min = None
        self._longitude_deg = None
        self._longitude_min = None
        self._bearing_deg = None
        self._speed = None

    def create_lcm(self):
        # If some part of Odom is uninitialized, return None
        if not (self._latitude_deg and
                self._latitude_min and
                self._longitude_deg and
                self._longitude_min and
                self._bearing_deg):
            return None

        msg = Odometry()
        msg.latitude_deg = self._latitude_deg
        msg.latitude_min = self._latitude_min
        msg.longitude_deg = self._longitude_deg
        msg.longitude_min = self._longitude_min
        msg.bearing_deg = self._bearing_deg
        msg.speed = -1
        return msg

    def copy_gps(self, gps):
        self._latitude_deg = gps._lat_deg
        self._latitude_min = gps._lat_min
        self._longitude_deg = gps._long_deg
        self._longitude_min = gps._long_min
        self._bearing_deg = gps._track_theta
        self._speed = gps._ground_speed
        return None


class raw_imu:
    def __init__(self):
        self._acc_x = None
        self._acc_y = None
        self._acc_z = None
        self._gyro_x = None
        self._gyro_y = None
        self._gyro_z = None
        self._mag_x = None
        self._mag_y = None
        self._mag_z = None
        self._bearing = None
        self._time_of_IMU = time.clock()

    def update_imu_bearing(self, message):
        self._acc_x = message.accel_x
        self._acc_y = message.accel_y
        self._acc_z = message.accel_z
        self._gyro_x = message.gyro_x
        self._gyro_y = message.gyro_y
        self._gyro_z = message.gyro_z
        self._mag_x = message.mag_x
        self._mag_y = message.mag_y
        self._mag_z = message.mag_z
        self._bearing = message.bearing
        self._time_of_IMU = time.clock()


class raw_gps:
    def __init__(self):
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None
        self._track_theta = None
        self._ground_speed = None
        self._time_of_GPS = time.clock()

    def updateGPS(self, message):
        self._lat_deg = message.latitude_deg
        self._lat_min = message.latitude_min
        self._long_deg = message.longitude_deg
        self._long_min = message.longitude_min
        self._track_theta = message.bearing_deg
        self._ground_speed = message.speed
        self._time_of_GPS = time.clock()


class nav_status:
    def __init__(self):
        self._navState = None
        self._time_of_status = time.clock()

    def update_nav_status(self, message):
        self._navState = message.nav_state
        self._time_of_IMU = time.clock()
