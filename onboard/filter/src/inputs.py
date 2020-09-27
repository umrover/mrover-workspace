import time
import numpy as np
from abc import ABC, abstractmethod
from copy import deepcopy
from .conversions import min2decimal, decimal2min


class Sensor(ABC):
    '''
    Abstract class for sensors

    @attribute bool fresh: current measurement is unused
    @attribute float last_fresh: timestamp of last update
    '''

    def __init__(self):
        self.fresh = False
        self.last_fresh = -1

    @abstractmethod
    def update(self, new_sensor):
        '''
        Updates the sensor with new sensor measurements

        @param LCM_msg new_sensor: new sensor LCM message

        Note: May explicitly checks for LCM struct attributes, add attributes as needed with
              new LCM structs
        '''
        pass

    @abstractmethod
    def ready(self):
        '''
        Returns true if all components and measurements are ready to use

        @return bool: sensor ready state
        '''
        pass


class SensorComponent(ABC):
    '''
    Abstract class for sensor components
    '''

    @abstractmethod
    def update(self, new_sensor):
        '''
        Updates the sensor component with the new sensor measurements

        @param LCM_msg new_sensor: new sensor LCM message

        Note: Explicitly checks for LCM struct attributes, add attributes as needed with
              new LCM structs
              Raises AttributeError if no compatible attributes are found
        '''
        pass

    @abstractmethod
    def ready(self):
        '''
        Returns true if all measurements are ready to use

        @return bool: component measurement ready state
        '''
        pass


class AccelComponent(SensorComponent):
    '''
    Class for acceleration sensor component

    @attribute float accel_x: relative acceleration (m/s^2) in x
    @attribute float accel_y: relative acceleration (m/s^2) in y
    @attribute float accel_z: relative acceleration (m/s^2) in z
    '''

    def __init__(self, filter_bias=1.0, threshold=0.0):
        self.accel_x = None
        self.accel_y = None
        self.accel_z = None
        self.filter_bias = filter_bias
        self.threshold_value = threshold
    
    def lowPass(self, new, old, bias):
        '''
        Returns the new value run through a low-pass filter

        @param float new: new value
        @param float old: old value
        @param float filter_bias: bias towards new value
        @return float: filtered new value
        '''
        if old is None:
            return new
        return new * bias + old * (1 - bias)
    
    def threshold(self, value, threshold_value):
        '''
        Returns value if |value| > threshold, else threshold

        @param float value: value to threshold
        @param float threshold: threshold value
        @return float: thresholded value
        '''
        return value if abs(value) > threshold_value else 0.0

    def update(self, new_accel_sensor):
        if hasattr(new_accel_sensor, "accel_x_g"):
            self.accel_x = self.lowPass(new_accel_sensor.accel_x_g * 9.8, self.accel_x, self.filter_bias)
            self.accel_x = self.threshold(self.accel_x, self.threshold_value)
            self.accel_y = self.lowPass(new_accel_sensor.accel_y_g * 9.8, self.accel_y, self.filter_bias)
            self.accel_y = self.threshold(self.accel_y, self.threshold_value)
            self.accel_z = self.lowPass(new_accel_sensor.accel_z_g * 9.8, self.accel_z, self.filter_bias)
            self.accel_z = self.threshold(self.accel_z, self.threshold_value)
        else:
            raise AttributeError("No acceleration attributes found")

    def ready(self):
        return self.accel_x is not None and self.accel_y is not None and \
            self.accel_z is not None

    def absolutify(self, bearing_deg, pitch_deg):
        '''
        Converts acceleration in x,y,z to acceleration in North,East,Z

        @param float bearing_deg: absolute bearing (decimal degrees East of North)
        @param float pitch_deg: absolute pitch (decimal degrees)
        @return dict: acceleration in North,East,Z
        '''
        if self.accel_x is None or bearing_deg is None or pitch_deg is None:
            return None

        absolute = {}
        absolute["north"] = self.accel_x * np.cos(np.radians(pitch_deg)) * \
            np.cos(np.radians(bearing_deg))
        absolute["east"] = self.accel_x * np.cos(np.radians(pitch_deg)) * \
            np.sin(np.radians(bearing_deg))
        absolute["z"] = self.accel_x * np.sin(np.radians(pitch_deg))
        return absolute


class VelComponent(SensorComponent):
    '''
    Class for velocity sensor component

    @attribute float ground_speed: directionless speed (m/s)
    '''

    def __init__(self):
        self.ground_speed = None

    def update(self, new_vel_sensor):
        # Raw speed is directionless, throw out negative values
        if hasattr(new_vel_sensor, "speed"):
            if new_vel_sensor.speed >= 0:
                self.ground_speed = new_vel_sensor.speed
        else:
            raise AttributeError("No speed attributes found")

    def ready(self):
        return self.ground_speed is not None

    def absolutify(self, bearing_deg):
        '''
        Separates raw ground speed into velocity in North,East
        '''
        if self.ground_speed is None or bearing_deg is None:
            return None

        # Should never happen due to the check in update method, but redundancy isn't bad
        if self.ground_speed < 0:
            return None

        absolute = {}
        absolute["north"] = self.ground_speed * np.cos(np.radians(bearing_deg))
        absolute["east"] = self.ground_speed * np.sin(np.radians(bearing_deg))
        return absolute


class PosComponent(SensorComponent):
    '''
    Class for GPS position sensor component

    @attribute float lat_deg: latitude (decimal degrees)
    @attribute float long_deg: longitude (decimal degrees)
    '''

    def __init__(self):
        self.lat_deg = None
        self.long_deg = None

    def update(self, new_pos_sensor):
        if hasattr(new_pos_sensor, "latitude_deg"):
            if hasattr(new_pos_sensor, "latitude_min"):
                self.lat_deg = min2decimal(new_pos_sensor.latitude_deg, new_pos_sensor.latitude_min)
                self.long_deg = min2decimal(new_pos_sensor.longitude_deg,
                                            new_pos_sensor.longitude_min)
            else:
                self.lat_deg = new_pos_sensor.latitude_deg
                self.long_deg = new_pos_sensor.longitude_deg
        else:
            raise AttributeError("No GPS attributes found")

    def ready(self):
        return self.lat_deg is not None and self.long_deg is not None

    def asDecimal(self):
        '''
        Returns latitude and longitude (decimal degrees)

        @return dict: GPS coordinates (decimal degrees)
        '''
        return {"lat": self.lat_deg, "long": self.long_deg}

    def asMinutes(self):
        '''
        Returns latitude and longitude (integer degrees, decimal minutes)

        @return dict: GPS coordinates (integer degrees, decimal minutes)
        '''
        minutes = {}
        minutes["lat_deg"], minutes["lat_min"] = decimal2min(self.lat_deg)
        minutes["long_deg"], minutes["long_min"] = decimal2min(self.long_deg)
        return minutes


class BearingComponent(SensorComponent):
    '''
    Class for bearing sensor component

    @attribute float bearing_deg: absolute bearing (decimal degrees East of North)
    '''

    def __init__(self):
        self.bearing_deg = None

    def update(self, new_bearing_sensor):
        if hasattr(new_bearing_sensor, "bearing"):
            self.bearing_deg = new_bearing_sensor.bearing
        elif hasattr(new_bearing_sensor, "bearing_deg"):
            if new_bearing_sensor.bearing_deg != 999.:
                self.bearing_deg = new_bearing_sensor.bearing_deg
        else:
            raise AttributeError("No bearing attributes found")

    def ready(self):
        return self.bearing_deg is not None


class AngVelComponent(SensorComponent):
    '''
    Class for angular velocity sensor component

    @attribute float ang_vel_x: relative angular velocity (dps) in x
    @attribute float ang_vel_y: relative angular velocity (dps) in y
    @attribute float ang_vel_z: relative angular velocity (dps) in z
    '''

    def __init__(self):
        self.ang_vel_x = None
        self.ang_vel_y = None
        self.ang_vel_z = None

    def update(self, new_ang_vel_sensor):
        if hasattr(new_ang_vel_sensor, "gyro_x_dps"):
            self.ang_vel_x = new_ang_vel_sensor.gyro_x_dps
            self.ang_vel_y = new_ang_vel_sensor.gyro_y_dps
            self.ang_vel_z = new_ang_vel_sensor.gyro_z_dps
        else:
            raise AttributeError("No angular velocity attributes found")

    def ready(self):
        return self.ang_vel_x is not None and self.ang_vel_y is not None and \
            self.ang_vel_z is not None


class MagComponent(SensorComponent):
    '''
    Class for magnetic sensor component

    @attribute float mag_x: relative magnetic strength (uT) in x
    @attribute float mag_y: relative magnetic strength (uT) in y
    @attribute float mag_z: relative magnetic strength (uT) in z
    '''

    def __init__(self):
        self.mag_x = None
        self.mag_y = None
        self.mag_z = None

    def update(self, new_mag_sensor):
        if hasattr(new_mag_sensor, "mag_x_uT"):
            self.mag_x = new_mag_sensor.mag_x_uT
            self.mag_y = new_mag_sensor.mag_y_uT
            self.mag_z = new_mag_sensor.mag_z_uT

    def ready(self):
        return self.mag_x is not None and self.mag_y is not None and self.mag_z is not None


class Imu(Sensor):
    '''
    Class for IMU sensor component

    @attribute AccelComponent accel: accelerometer component of IMU
    @attribute BearingComponent bearing: bearing component of IMU
    @attribute AngVelComponent gyro: gyroscope component of IMU
    @attribute MagComponent mag: magnetometer component of IMU
    @attribute float roll_deg: absolute roll (decimal degrees)
    @attribute float pitch_deg: absolute pitch (decimal degrees)
    @attribute float yaw_deg: absolute yaw (decimal degrees)
    '''

    def __init__(self, accel_filter_bias=1.0, accel_threshold=0.0):
        super().__init__()
        self.accel = AccelComponent(accel_filter_bias, accel_threshold)
        self.bearing = BearingComponent()
        self.gyro = AngVelComponent()
        self.mag = MagComponent()
        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_deg = None

    def update(self, new_imu):
        # Hold onto old values in case we need to revert update
        old_accel = deepcopy(self.accel)
        old_bearing = deepcopy(self.bearing)
        old_gyro = deepcopy(self.gyro)
        old_mag = deepcopy(self.mag)
        old_roll_deg = self.roll_deg
        old_pitch_deg = self.pitch_deg
        old_yaw_deg = self.yaw_deg

        try:
            self.accel.update(new_imu)
            self.bearing.update(new_imu)
            self.gyro.update(new_imu)
            self.mag.update(new_imu)

            if hasattr(new_imu, "roll_rad"):
                if np.isnan(new_imu.roll_rad):
                    print("IMU RPY NaN, setting to zero")
                    self.roll_deg = 0
                    self.pitch_deg = 0
                    self.yaw_deg = 0
                else:
                    self.roll_deg = np.degrees(new_imu.roll_rad)
                    self.pitch_deg = np.degrees(new_imu.pitch_rad)
                    self.yaw_deg = np.degrees(new_imu.yaw_rad)
            else:
                raise AttributeError("No roll/pitch/yaw attributes found")
            self.fresh = True
            self.last_fresh = time.time()
        except AttributeError as e:
            print(e)
            # Revert to old values on error
            self.accel = old_accel
            self.bearing = old_bearing
            self.gyro = old_gyro
            self.mag = old_mag
            self.roll_deg = old_roll_deg
            self.pitch_deg = old_pitch_deg
            self.yaw_deg = old_yaw_deg
            self.fresh = False

    def ready(self):
        return self.accel.ready() and self.bearing.ready() and self.gyro.ready() and \
            self.mag.ready() and self.roll_deg is not None and self.pitch_deg is not None and \
            self.yaw_deg is not None


class Gps(Sensor):
    '''
    Class for GPS data

    @attribute VelComponent vel: velocity component of GPS
    @attribute PosComponent pos: position component of GPS
    @attribute BearingComponent bearing: bearing component of GPS
    '''

    def __init__(self):
        super().__init__()
        self.vel = VelComponent()
        self.pos = PosComponent()
        self.bearing = BearingComponent()

    def update(self, new_gps):
        # Hold onto old values in case we need to revert update
        old_vel = deepcopy(self.vel)
        old_pos = deepcopy(self.pos)
        old_bearing = deepcopy(self.bearing)

        try:
            self.vel.update(new_gps)
            self.pos.update(new_gps)
            self.bearing.update(new_gps)
            self.fresh = True
            self.last_fresh = time.time()
        except AttributeError as e:
            print(e)
            # Revert to old values on error
            self.vel = old_vel
            self.pos = old_pos
            self.bearing = old_bearing
            self.fresh = False

    def ready(self):
        return self.vel.ready() and self.pos.ready()
