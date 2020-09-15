from rover_common import aiolcm
from rover_msgs import IMUData, GPS
import csv
import os
import numpy as np
import time
import math
import json
from .pathGenerator import PathGenerator


class Simulator:

    def __init__(self):
        # Fetch constants
        config_path = os.getenv('MROVER_CONFIG')
        sim_config_path = config_path + "/config_filter/simConfig.json"
        with open(sim_config_path, "r") as configJson:
            config = json.load(configJson)
            self.GPS_UPDATE_RATE_MILLIS = config['gps_update_rate_millis']
            self.IMU_UPDATE_RATE_MILLIS = config['imu_update_rate_millis']
            self.DT_MILLIS = config['dt_s'] * 1000
            self.CSV_MODE = config['csv_mode']
        config_path += "/config_filter/config.json"
        with open(config_path, "r") as configJson:
            config = json.load(configJson)
            self.SEND_RATE_MILLIS = config['UpdateRate'] * 1000
            self.REF_LAT = config["RefCoords"]["lat"]
            self.REF_LONG = config["RefCoords"]["long"]
        self.GPS_DTS = int(self.GPS_UPDATE_RATE_MILLIS / self.DT_MILLIS)
        self.IMU_DTS = int(self.IMU_UPDATE_RATE_MILLIS / self.DT_MILLIS)

        self.path_generator = PathGenerator(self.REF_LAT, self.REF_LONG)
        path_out = self.path_generator.run()
        self.truth = path_out['truth']
        self.noisy = path_out['noisy']

        self.lcm = aiolcm.AsyncLCM()

        self.millis = time.time() * 1000
        self.timesteps = 0

    def recordTruth(self):
        os.makedirs(os.path.join(os.getcwd(), 'onboard', 'filter', 'logs'),
                    exist_ok=True)
        with open(os.path.join('onboard', 'filter', 'logs', 'truthLog.csv'), mode=self.CSV_MODE)\
                as log:
            writer = csv.writer(log)
            writer.writerow(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing', 'speed'])
            for i in range(len(self.truth['gps_north'])):
                lat_min, lat_deg = math.modf(self.truth['gps_north'][i])
                lat_min *= 60
                long_min, long_deg = math.modf(self.truth['gps_east'][i])
                long_min *= 60
                bearing = self.truth['bearing'][i]
                speed = self.truth['vel_total'][i]
                writer.writerow([lat_deg, lat_min, long_deg, long_min, bearing, speed])

    def sendTimestep(self):
        # send at update rate
        if (time.time() * 1000 - self.millis) < self.SEND_RATE_MILLIS:
            return

        imu = IMUData()
        gps = GPS()

        imu.accel_x_g = self.noisy['accel_x'][self.timesteps]/9.8
        imu.accel_y_g = self.noisy['accel_y'][self.timesteps]/9.8
        imu.accel_z_g = self.noisy['accel_z'][self.timesteps]/9.8
        imu.gyro_x_dps = imu.gyro_y_dps = imu.gyro_z_dps = imu.mag_x_uT = imu.mag_y_uT = \
            imu.mag_z_uT = imu.roll_rad = imu.yaw_rad = 0
        imu.bearing_deg = self.noisy['bearing'][self.timesteps]
        imu.pitch_rad = np.radians(self.noisy['pitch'][self.timesteps])

        gps.latitude_min, gps.latitude_deg = math.modf(self.noisy['gps_north'][self.timesteps])
        gps.latitude_deg = int(gps.latitude_deg)
        gps.latitude_min *= 60
        gps.longitude_min, gps.longitude_deg = math.modf(self.noisy['gps_east'][self.timesteps])
        gps.longitude_deg = int(gps.longitude_deg)
        gps.longitude_min *= 60
        gps.bearing_deg = self.noisy['bearing'][self.timesteps]
        gps.speed = self.noisy['vel_total'][self.timesteps]
        if gps.speed < 0:
            gps.speed = 0
        gps.quality = 0

        if self.timesteps % self.GPS_DTS == 0:
            self.lcm.publish('/gps', gps.encode())
            # print('Sending GPS')

        if self.timesteps % self.IMU_DTS == 0:
            self.lcm.publish('/imu', imu.encode())
            # print(self.timesteps / self.IMU_DTS)
            # print('Sending IMU')

        self.millis = time.time() * 1000
        self.timesteps += 1

    def run(self):
        self.timesteps = 0
        print("Running...")
        while self.timesteps < len(self.noisy['gps_north']):
            self.sendTimestep()
        print("Done!")


def main():
    print("Starting simulation...")
    sim = Simulator()
    sim.recordTruth()
    sim.run()
    while True:
        cont = input("continue? y|n ")
        if cont == 'n':
            break
        sim.recordTruth()
        sim.run()


if __name__ == '__main__':
    main()
