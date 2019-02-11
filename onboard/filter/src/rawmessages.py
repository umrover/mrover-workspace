import time


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
        self._track_angle = None
        self._ground_speed = None
        self._time_of_GPS = time.clock()

    def updateGPS(self, message):
        self._lat_deg = message.latitude_deg
        self._lat_min = message.latitude_min
        self._long_deg = message.longitude_deg
        self._long_min = message.longitude_min
        self._track_angle = message.bearing_deg
        self._ground_speed = message.speed
        self._time_of_GPS = time.clock()


class nav_status:
    def __init__(self):
        self._navState = None
        self._time_of_status = time.clock()

    def update_nav_status(self, message):
        self._navState = message.nav_state
        self._time_of_IMU = time.clock()
