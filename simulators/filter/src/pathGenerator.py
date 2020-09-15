import numpy as np
import scipy.integrate
import json
from os import getenv
import random
import math

'''
Goal is to simulate the sensor inputs one could expect to see for a given path

Starts by randomly generating a number of acceleration readings under various constraints

Integrates over acceleration readings to create "true path"

Applies normal distribution to generated acceleration and gps readings

Result is a correct path and a noisy path to filter.

Stretch Goal is plot noisy, filtered, and true paths and calculate deviation of
filtered path from truth
'''


class PathGenerator:

    def __init__(self, ref_lat, ref_long):
        # Fetch constants
        config_path = getenv('MROVER_CONFIG')
        config_path += "/config_filter/simConfig.json"
        with open(config_path, "r") as configJson:
            config = json.load(configJson)
            self.DT_S = config['dt_s']
            self.GPS_LAT_STDEV_METERS = config['gps_lat_stdev_meters']
            self.GPS_LONG_STDEV_METERS = config['gps_long_stdev_meters']
            self.GPS_VEL_STDEV_METERS = config['gps_vel_stdev_meters']
            self.GPS_BEARING_STDEV_DEGS = config['gps_bearing_stdev_degs']
            self.IMU_ACCEL_STDEV_METERS = config['imu_accel_stdev_meters']
            self.IMU_BEARING_STDEV_DEGS = config['imu_bearing_stdev_degs']
            self.IMU_PITCH_STDEV_DEGS = config['imu_bearing_stdev_degs']
            self.MAX_JERK = config['max_jerk'] * self.DT_S
            self.MAX_ACCEL = config['max_accel'] * self.DT_S
            self.MIN_ACCEL = config['min_accel'] * self.DT_S
            self.MAX_ROT_VEL_DEGS = np.degrees(config['max_rot_vel_rads']) * self.DT_S
            self.MAX_PITCH_VEL_DEGS = np.degrees(config['max_pitch_vel_rads']) * self.DT_S
            self.END_TIME = config['end_time']
        self.MAX_READINGS = np.int_(self.END_TIME / self.DT_S)
        self.REF_LAT = ref_lat
        self.REF_LONG = ref_long

    # generates accelerations in m/s2
    def pointGen(self):
        last_point = random.uniform(self.MIN_ACCEL, self.MAX_ACCEL)
        num_points = 1
        yield np.float64(last_point)
        while num_points < self.MAX_READINGS:
            # new_point = random.uniform(max(last_point - self.MAX_JERK, self.MIN_ACCEL),
            #                            min(last_point + self.MAX_JERK, self.MAX_ACCEL))
            new_point = np.random.uniform(max(last_point - 0.2*self.MAX_JERK, self.MIN_ACCEL),
                                          min(last_point + self.MAX_JERK, self.MAX_ACCEL))
            num_points += 1
            last_point = new_point
            yield np.float64(new_point)

    # generates angles in degrees
    def angleGen(self, delta, initial=None, _min=None, _max=None):
        if initial is None:
            if _min is None or _max is None:
                last_angle = random.uniform(0, 360)
            else:
                last_angle = random.uniform(_min, _max)
        else:
            last_angle = initial
        num_angles = 1
        yield np.float64(last_angle % 360)
        while num_angles < self.MAX_READINGS:
            if _min is None or _max is None:
                new_angle = random.uniform(last_angle - delta,
                                           last_angle + delta)
            else:
                new_angle = random.uniform(max((last_angle - delta), _min),
                                           min((last_angle + delta), _max))
            num_angles += 1

            last_angle = new_angle
            yield np.float64(new_angle % 360)

    # generates a true path (m/s)
    def generateTruth(self):

        # generate control inputs
        accel_points_x = np.fromiter(self.pointGen(), np.float64, count=self.MAX_READINGS)

        bearing_angles_degs = np.fromiter(self.angleGen(self.MAX_ROT_VEL_DEGS), np.float64,
                                          count=self.MAX_READINGS-1)
        pitch_angles_degs = np.fromiter(self.angleGen(self.MAX_PITCH_VEL_DEGS, _min=-60, _max=60),
                                        np.float64, count=self.MAX_READINGS-1)

        # create sin cos arrays for absolutification
        bearing_cos = [math.cos(np.radians(i)) for i in bearing_angles_degs]
        bearing_sin = [math.sin(np.radians(i)) for i in bearing_angles_degs]
        pitch_cos = [math.cos(np.radians(i)) for i in pitch_angles_degs]

        # generate absolute accelerations
        # accel_points_north = np.multiply(accel_points_x, np.multiply(pitch_cos, bearing_cos))
        # accel_points_west = -(np.multiply(accel_points_x, np.multiply(pitch_cos, bearing_sin)))

        # generate velocities
        vel_points_x = scipy.integrate.cumtrapz(accel_points_x, dx=self.DT_S)
        vel_points_north = np.multiply(vel_points_x, np.multiply(pitch_cos, bearing_cos))
        vel_points_east = np.multiply(vel_points_x, np.multiply(pitch_cos, bearing_sin))

        gps_points_north = scipy.integrate.cumtrapz(vel_points_north, dx=self.DT_S)
        gps_points_north = meters2lat(gps_points_north, ref_lat=self.REF_LAT)
        gps_points_east = scipy.integrate.cumtrapz(vel_points_east, dx=self.DT_S)
        gps_points_east = meters2long(gps_points_east, gps_points_north, ref_long=self.REF_LONG)

        return {
            "accel_x": accel_points_x,
            "accel_y": np.zeros(self.MAX_READINGS),
            "accel_z": np.zeros(self.MAX_READINGS),
            "vel_north": vel_points_north,
            "vel_east": vel_points_east,
            "vel_total": vel_points_x,
            "gps_north": gps_points_north,
            "gps_east": gps_points_east,
            "bearing": bearing_angles_degs,
            "pitch": pitch_angles_degs
        }

    # applies noise to a true path
    def applyNoise(self, truth):

        noisy_accel_points_x = np.random.normal(truth['accel_x'], self.IMU_ACCEL_STDEV_METERS)
        noisy_accel_points_y = np.random.normal(np.zeros(self.MAX_READINGS),
                                                self.IMU_ACCEL_STDEV_METERS)
        noisy_accel_points_z = np.random.normal(np.zeros(self.MAX_READINGS),
                                                self.IMU_ACCEL_STDEV_METERS)

        noisy_vel_points_total = np.random.normal(truth['vel_total'], self.GPS_VEL_STDEV_METERS)

        noisy_gps_points_north = np.random.normal(truth['gps_north'],
                                                  meters2lat(self.GPS_LAT_STDEV_METERS))
        noisy_gps_points_east = np.random.normal(truth['gps_east'],
                                                 [abs(i) for i in
                                                  meters2long(self.GPS_LONG_STDEV_METERS,
                                                              truth['gps_north'])])

        noisy_bearing_angles_degs = np.random.normal(truth['bearing'], self.IMU_BEARING_STDEV_DEGS)
        noisy_pitch_angles_degs = np.random.normal(truth['pitch'], self.IMU_PITCH_STDEV_DEGS)

        return {
            "accel_x": noisy_accel_points_x,
            "accel_y": noisy_accel_points_y,
            "accel_z": noisy_accel_points_z,
            "vel_total": noisy_vel_points_total,
            "gps_north": noisy_gps_points_north,
            "gps_east": noisy_gps_points_east,
            "bearing": noisy_bearing_angles_degs,
            "pitch": noisy_pitch_angles_degs
        }

    def generator(self):

        while True:
            self.truth = self.generateTruth()
            self.noisy = self.applyNoise(self.truth)

            yield {"truth": self.truth, "noisy": self.noisy}

    def run(self):
        # returns true and noisy paths. generates new path if new_path
        return next(self.generator())


EARTH_RADIUS_M = 6371000


def lat2meters(lat, ref_lat=0):
    '''
    Converts degrees latitude to meters North/South using a nearby ref_lat to
    reduce conversion error

    @param float/ndarray lat: absolute degrees latitude
    @optional ref_lat: reference latitude used to reduce conversion error
    @return float/ndarray: meters North/South relative to ref_lat
    '''
    return np.radians(lat - ref_lat) * EARTH_RADIUS_M


def meters2lat(meters, ref_lat=0):
    '''
    Converts meters North/South to degrees latitude, adding ref_lat to counteract
    its use in lat2meters

    @param float/ndarray meters: meters North/South relative to ref_lat
    @optional float ref_lat: reference latitude used to reduce conversion error
    @return float/ndarray: absolute degrees latitude
    '''
    return np.degrees(meters / EARTH_RADIUS_M) + ref_lat


def long2meters(long, lat, ref_long=0):
    '''
    Converts degrees longitude to meters East/West using a nearby ref_long to reduce
    conversion error

    @param float/ndarray long: absolute degrees longitude
    @param float/ndarray lat: absolute degrees latitude
    @optional ref_long: reference longitude used to reduce conversion error
    @return float/ndarray: meters East/West relative to ref_long
    '''
    lat_cos = np.cos(np.radians(lat))
    scaled_long = np.multiply(long - ref_long, lat_cos)
    return np.radians(scaled_long) * EARTH_RADIUS_M


def meters2long(meters, lat, ref_long=0):
    '''
    Converts meters East/West to degrees longitude, adding ref_long to counteract its
    use in long2meters

    @param float/ndarray meters: meters East/West relative to ref_long
    @param float/ndarray lat: absolute degrees latitude
    @optional float ref_long: reference longitude used to reduce conversion error
    @return float/ndarray: absolute degrees longitude
    '''
    lat_cos = np.cos(np.radians(lat))
    scaled_long = np.degrees(meters / EARTH_RADIUS_M)
    return (scaled_long / lat_cos) + ref_long
