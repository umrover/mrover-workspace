import csv
import json
import os
import time

from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import IMUData, GPS, Odometry


class Logger:

    def __init__(self):
        # Read in options from logConfig
        config_path = os.getenv('MROVER_CONFIG')
        config_path += "/config_filter/logConfig.json"
        with open(config_path, "r") as config:
            self.logConfig = json.load(config)

        config_path = os.getenv('MROVER_CONFIG')
        config_path += "/config_filter/config.json"
        with open(config_path, "r") as config:
            self.filterConfig = json.load(config)

        # Make logs if they don't already exist
        os.makedirs(os.path.join(os.getcwd(), 'onboard', 'filter', 'logs'),
                    exist_ok=True)

        # Create files and write headers
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'gps')
        self.write(['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y',
                    'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'bearing'], 'imu')
        self.write(['nav_state', 'nav_state_name', 'completed_wps',
                    'missed_wps', 'total_wps', 'found_tbs', 'total_tbs'],
                   'navStatus')
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'phone')
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'odom')
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'movAvg')
        self.write(['Q', 'FilterType', 'P_initial', 'R', 'dt', 'UpdateRate'], 'config')

        P_initial_str = str(self.filterConfig['P_initial']).replace(',', ' ')
        R_str = str(self.filterConfig['R']).replace(',', ' ')
        self.write([self.filterConfig['Q'], self.filterConfig['FilterType'],
                    P_initial_str, R_str, self.filterConfig['dt'],
                    self.filterConfig['UpdateRate']], 'config')

        # Subscribe to LCM channels
        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self.gps_callback)
        self.lcm.subscribe("/imu", self.imu_callback)
        self.lcm.subscribe("/odometry", self.odom_callback)

        # Initialize sensor timestamps
        self.gps_millis = time.time() * 1000
        self.imu_millis = time.time() * 1000
        self.odom_millis = time.time() * 1000

    def write(self, contents, type):
        # Writes contents to the log specified by type
        # with open(self.file_path + type + 'Log.csv', 'w') as log:
        with open(os.path.join('onboard', 'filter', 'logs', type + 'Log.csv'),
                  mode=self.logConfig['mode']) as log:
            writer = csv.writer(log)
            writer.writerow(contents)

    def gps_callback(self, channel, msg):
        gps = GPS.decode(msg)
        if (time.time()*1000 - self.gps_millis) > \
                self.logConfig['rate_millis']['gps']:
            self.write([gps.latitude_deg, gps.latitude_min, gps.longitude_deg,
                        gps.longitude_min, gps.bearing_deg, gps.speed], 'gps')
            self.gps_millis = time.time()*1000

    def imu_callback(self, channel, msg):
        imu = IMUData.decode(msg)
        if (time.time()*1000 - self.imu_millis) > \
                self.logConfig['rate_millis']['imu']:
            self.write([imu.accel_x_g, imu.accel_y_g, imu.accel_z_g, imu.gyro_x_dps,
                        imu.gyro_y_dps, imu.gyro_z_dps, imu.mag_x_uT, imu.mag_y_uT,
                        imu.mag_z_uT, imu.bearing_deg], 'imu')
            self.imu_millis = time.time()*1000

    def odom_callback(self, channel, msg):
        odom = Odometry.decode(msg)
        if (time.time()*1000 - self.odom_millis) > \
                self.logConfig['rate_millis']['odom']:
            self.write([odom.latitude_deg, odom.latitude_min,
                        odom.longitude_deg, odom.longitude_min,
                        odom.bearing_deg, odom.speed], 'odom')
            self.odom_millis = time.time()*1000


def main():
    logger = Logger()
    run_coroutines(logger.lcm.loop())


if __name__ == "__main__":
    main()
