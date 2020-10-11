import csv
import json
import os
import time

from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import IMUData, GPS, Odometry
import matplotlib.pyplot as plt

import numpy as np
import math
import asyncio


EARTH_RADIUS_M = 6371000
REF_LAT = 42.2766
REF_LONG = -83.7382


def lat2meters(lat, ref_lat=0):
    '''
    Converts degrees latitude to meters North/South using a nearby ref_lat to
    reduce conversion error

    @param float/ndarray lat: absolute degrees latitude
    @optional ref_lat: reference latitude used to reduce conversion error
    @return float/ndarray: meters North/South relative to ref_lat
    '''
    return np.radians(lat - ref_lat) * EARTH_RADIUS_M

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

def decimal2min(decimal):
    '''
    Converts decimal degrees to integer degrees and decimal minutes

    @param float decimal: decimal degrees
    @return int: integer degrees
    @return float: decimal minutes
    '''
    min, deg = math.modf(decimal)
    return int(deg), min * 60


def min2decimal(deg, min):
    '''
    Converts integer degrees and decimal minutes to decimal degrees

    @param int deg:  integer degrees
    @param float min: decimal minutes
    @return float: decimal degrees
    '''
    return deg + min / 60

class Plotter:
    def __init__(self):
        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self.gps_callback)
        self.lcm.subscribe("/odometry", self.odom_callback)
        self.gps_millis = time.time() * 1000
        self.odom_millis = time.time() * 1000
        self.current_gps = None
        self.current_odom = None
        # plt.axis([-1000, 1000, -1000, 1000])

    def gps_callback(self, channel, msg):
        gps = GPS.decode(msg)
        if (time.time()*1000 - self.gps_millis) > 0.1:
            lat = min2decimal(gps.latitude_deg, gps.latitude_min)
            long = min2decimal(gps.longitude_deg, gps.longitude_min)
            long = long2meters(long, lat, ref_long=REF_LONG)
            lat = lat2meters(lat, REF_LAT)
            self.current_gps = (lat, long)
            self.gps_millis = time.time()*1000
    
    def odom_callback(self, channel, msg):
        odom = Odometry.decode(msg)
        if (time.time()*1000 - self.odom_millis) > 0.1:
            lat = min2decimal(odom.latitude_deg, odom.latitude_min)
            long = min2decimal(odom.longitude_deg, odom.longitude_min)
            long = long2meters(long, lat, ref_long=REF_LONG)
            lat = lat2meters(lat, REF_LAT)
            self.current_odom = (lat, long)
            self.odom_millis = time.time()*1000
    
    async def livePlot(self):
        while True:
            if self.current_gps is not None and self.current_odom is not None:
                plt.scatter(self.current_gps[0], self.current_gps[1], color="red")
                plt.scatter(self.current_odom[0], self.current_odom[1], color="black")
                plt.pause(0.05)
            await asyncio.sleep(0.1)

def main():
    plotter = Plotter()
    # plt.show()
    run_coroutines(plotter.lcm.loop(), plotter.livePlot())

if __name__ == "__main__":
    main()


# class Logger:

#     def __init__(self):
#         # Read in options from logConfig
#         config_path = os.getenv('MROVER_CONFIG')
#         config_path += "/config_filter/logConfig.json"
#         with open(config_path, "r") as config:
#             self.logConfig = json.load(config)

#         config_path = os.getenv('MROVER_CONFIG')
#         config_path += "/config_filter/config.json"
#         with open(config_path, "r") as config:
#             self.filterConfig = json.load(config)

#         # Make logs if they don't already exist
#         os.makedirs(os.path.join(os.getcwd(), 'onboard', 'filter', 'logs'),
#                     exist_ok=True)

#         # Create files and write headers
#         self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
#                     'speed'], 'gps')
#         self.write(['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y',
#                     'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'bearing'], 'imu')
#         self.write(['nav_state', 'nav_state_name', 'completed_wps',
#                     'missed_wps', 'total_wps', 'found_tbs', 'total_tbs'],
#                    'navStatus')
#         self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
#                     'speed'], 'phone')
#         self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
#                     'speed'], 'odom')
#         self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
#                     'speed'], 'movAvg')
#         self.write(['Q', 'FilterType', 'P_initial', 'R', 'dt', 'UpdateRate'], 'config')

#         P_initial_str = str(self.filterConfig['P_initial']).replace(',', ' ')
#         R_str = str(self.filterConfig['R']).replace(',', ' ')
#         self.write([self.filterConfig['Q'], self.filterConfig['FilterType'],
#                     P_initial_str, R_str, self.filterConfig['dt'],
#                     self.filterConfig['UpdateRate']], 'config')

#         # Subscribe to LCM channels
#         self.lcm = aiolcm.AsyncLCM()
#         self.lcm.subscribe("/gps", self.gps_callback)
#         self.lcm.subscribe("/imu", self.imu_callback)
#         self.lcm.subscribe("/odometry", self.odom_callback)

#         # Initialize sensor timestamps
#         self.gps_millis = time.time() * 1000
#         self.imu_millis = time.time() * 1000
#         self.odom_millis = time.time() * 1000

#     def write(self, contents, type):
#         # Writes contents to the log specified by type
#         # with open(self.file_path + type + 'Log.csv', 'w') as log:
#         with open(os.path.join('onboard', 'filter', 'logs', type + 'Log.csv'),
#                   mode=self.logConfig['mode']) as log:
#             writer = csv.writer(log)
#             writer.writerow(contents)

#     def gps_callback(self, channel, msg):
#         gps = GPS.decode(msg)
#         if (time.time()*1000 - self.gps_millis) > \
#                 self.logConfig['rate_millis']['gps']:
#             self.write([gps.latitude_deg, gps.latitude_min, gps.longitude_deg,
#                         gps.longitude_min, gps.bearing_deg, gps.speed], 'gps')
#             self.gps_millis = time.time()*1000

#     def imu_callback(self, channel, msg):
#         imu = IMUData.decode(msg)
#         if (time.time()*1000 - self.imu_millis) > \
#                 self.logConfig['rate_millis']['imu']:
#             self.write([imu.accel_x_g, imu.accel_y_g, imu.accel_z_g, imu.gyro_x_dps,
#                         imu.gyro_y_dps, imu.gyro_z_dps, imu.mag_x_uT, imu.mag_y_uT,
#                         imu.mag_z_uT, imu.bearing_deg], 'imu')
#             self.imu_millis = time.time()*1000

#     def odom_callback(self, channel, msg):
#         odom = Odometry.decode(msg)
#         if (time.time()*1000 - self.odom_millis) > \
#                 self.logConfig['rate_millis']['odom']:
#             self.write([odom.latitude_deg, odom.latitude_min,
#                         odom.longitude_deg, odom.longitude_min,
#                         odom.bearing_deg, odom.speed], 'odom')
#             self.odom_millis = time.time()*1000


# def main():
#     logger = Logger()
#     run_coroutines(logger.lcm.loop())


# if __name__ == "__main__":
#     main()
