import math


def calc_mov_avg(readings, degree=1):
    """
    Calculates a moving average with a degree 'degree' on the list of
    data in 'readings'. The default degree is 1, a linear moving average.
    """
    if not readings:
        return None
    total_readings = 0
    total_weights = 0
    for i in range(len(readings)):
        weight = (i + 1) ** degree
        total_readings += weight * readings[i]
        total_weights += weight
    return total_readings / total_weights


class RawIMU:
    """Class for holding and performing calculations on the raw IMU data."""

    def __init__(self, num_prev_bearings=5):
        """Initialize IMU data."""
        # accelerometer
        self._acc_x = None
        self._acc_y = None
        self._acc_z = None
        # gyroscope
        self._gyro_x = None
        self._gyro_y = None
        self._gyro_z = None
        # magnetometer
        self._mag_x = None
        self._mag_y = None
        self._mag_z = None
        # IMU's bearing output
        self._bearing = None
        self._prev_bearings = []
        # number of readings to use for moving average
        self._num_prev_bearings = num_prev_bearings

        self._pitch = None
        self._roll = None
        self._yaw = None

    def __str__(self):
        return "acc_x: {} acc_y:{} acc_z:{}".format(self._acc_x,
                                                    self._acc_y,
                                                    self._acc_z)

    def update_imu(self, new_imu):
        """Updates IMU data with new IMU lcm message."""
        self._acc_x = new_imu.accel_x
        self._acc_y = new_imu.accel_y
        self._acc_z = new_imu.accel_z
        self._gyro_x = new_imu.gyro_x
        self._gyro_y = new_imu.gyro_y
        self._gyro_z = new_imu.gyro_z
        self._mag_x = new_imu.mag_x
        self._mag_y = new_imu.mag_y
        self._mag_z = new_imu.mag_z
        self._bearing = new_imu.bearing
        self._prev_bearings.append(self._bearing)
        if len(self._prev_bearings) > self._num_prev_bearings:
            self._prev_bearings.pop(0)

        self.calc_pitch()
        self.calc_roll()
        self.calc_yaw()

    def bearing_mov_avg(self):
        """
        Calculates a weighted linear moving average over the previous 5
        bearing readings.
        """
        return calc_mov_avg(self._prev_bearings)

    def calc_pitch(self):
        """Calculates the rover's pitch based on the accelerometer."""
        acc_yz = math.sqrt(math.pow(self._acc_y, 2) + math.pow(self._acc_z, 2))
        self._pitch = 180 * math.atan2(self._acc_x, acc_yz) / math.pi

    def calc_roll(self):
        """Calculates the rover's roll based on the accelerometer."""
        acc_xz = math.sqrt(math.pow(self._acc_x, 2) + math.pow(self._acc_z, 2))
        self._roll = 180 * math.atan2(self._acc_y, acc_xz) / math.pi

    def calc_yaw(self):
        """
        Calculates the yaw based on the magnetometer readings, pitch,
        and roll.
        """
        cos_pitch = math.cos(self._pitch)
        sin_pitch = math.sin(self._pitch)
        cos_roll = math.cos(self._roll)
        sin_roll = math.sin(self._roll)
        partial_mag_xx = self._mag_x * cos_pitch
        partial_mag_xy = self._mag_y * sin_roll * sin_pitch
        partial_mag_xz = self._mag_z * cos_roll * sin_pitch
        mag_x = partial_mag_xx + partial_mag_xy + partial_mag_xz
        mag_y = self._mag_y * cos_roll - self._mag_z * sin_roll
        self._yaw = 180 * math.atan2(-mag_y, mag_x) / math.pi


class RawGPS:
    """Class for holding and performing calculations on the raw GPS data."""

    def __init__(self, num_prev_track_angles=5):
        """Initialize GPS data."""
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None
        self._track_angle = None
        self._ground_speed = None
        self._prev_track_angles = []
        self._num_prev_track_angles = num_prev_track_angles

    def update_gps(self, new_gps):
        """Updates the gps data with the new lcm message."""
        self._lat_deg = new_gps.latitude_deg
        self._lat_min = new_gps.latitude_min
        self._long_deg = new_gps.longitude_deg
        self._long_min = new_gps.longitude_min
        self._track_angle = new_gps.bearing_deg
        self._prev_track_angles.append(self._track_angle)
        if len(self._prev_track_angles) > self._num_prev_track_angles:
            self._prev_track_angles.pop(0)
        self._ground_speed = new_gps.speed

    def track_mov_avg(self):
        """
        Calculates a weighted linear moving average over the previous 5
        track angle readings.
        """
        return calc_mov_avg(self._prev_track_angles)


class RawSensorPackage:
    """Class for sensor package data and calculations."""

    def __init__(self):
        """Initialize sensor package data."""
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None
        self._bearing = None

    def update(self, sensor_package):
        """Update sensor package data with new LCM data."""
        self._lat_deg = sensor_package.latitude_deg
        self._lat_min = sensor_package.latitude_min
        self._long_deg = sensor_package.longitude_deg
        self._long_min = sensor_package.longitude_min
        self._bearing = sensor_package.bearing


class RawNavStatus:
    """Class for containing the nav_status information."""

    def __init__(self):
        """Initialize nav status data."""
        self._navState = None

    def update_nav_status(self, new_nav_status):
        """Updates the nav status with the new nav status message."""
        self._navState = new_nav_status.nav_state_name
