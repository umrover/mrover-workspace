import json
import time
import asyncio
import numpy as np
from os import getenv
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import IMUData, GPS, Odometry
from .inputs import Gps, Imu
from .linearKalman import LinearKalmanFilter, QDiscreteWhiteNoise
from .conversions import meters2lat, meters2long, lat2meters, long2meters, \
                        decimal2min, min2decimal


class StateEstimate:
    '''
    Class for current state estimate

    @attribute dict pos: current position estimate (integer degrees, decimal minutes)
    @attribute dict vel: current velocity estimate (m/s)
    @attribute float bearing_deg: current bearing estimate (decimal degrees East of North)
    @attribute float ref_lat: reference latitude used to reduce conversion error
    @attribute float ref_long: reference longitude used to reduce conversion error
    '''

    def __init__(self, lat_deg=None, lat_min=None, vel_north=None,
                 long_deg=None, long_min=None, vel_east=None, bearing_deg=None,
                 ref_lat=0, ref_long=0):
        '''
        Initalizes state variable values

        @optional int lat_deg: latitude integer degrees
        @optional float lat_min: latitude decimal minutes
        @optional float vel_north: velocity North (m/s)
        @optional int long_deg: longitude integer degrees
        @optional float long_min: longitude decimal minutes
        @optional float vel_east: velocity East (m/s)
        @optional float bearing_deg: absolute bearing (decimal degrees East of North)
        @optional float ref_lat: reference latitude used to reduce conversion error
        @optional float ref_long: reference longitude used to reduce conversion error
        '''
        self.pos = {"lat_deg": lat_deg, "lat_min": lat_min, "long_deg": long_deg,
                    "long_min": long_min}
        self.vel = {"north": vel_north, "east": vel_east}
        self.bearing_deg = bearing_deg
        self.ref_lat = ref_lat
        self.ref_long = ref_long

    def posToMeters(self):
        '''
        Returns the current position estimate converted to meters

        @return dict: current position estimate (meters)
        '''
        pos_meters = {}
        pos_meters["long"] = min2decimal(self.pos["long_deg"], self.pos["long_min"])
        pos_meters["lat"] = min2decimal(self.pos["lat_deg"], self.pos["lat_min"])
        pos_meters["long"] = long2meters(pos_meters["long"], pos_meters["lat"],
                                         ref_long=self.ref_long)
        pos_meters["lat"] = lat2meters(pos_meters["lat"], ref_lat=self.ref_lat)
        return pos_meters

    def asLKFInput(self):
        '''
        Returns the state estimate as an ndarray for LKF input

        @return ndarray: state vector
        '''
        pos_meters = self.posToMeters()
        return np.array([pos_meters["lat"], self.vel["north"], pos_meters["long"],
                         self.vel["east"]])

    def updateFromLKF(self, lkf_out):
        '''
        Updates state estimate from the filter output

        @param ndarray lkf_out: LKF state vector
        '''
        lat_decimal_deg = meters2lat(lkf_out[0], ref_lat=self.ref_lat)
        self.pos["lat_deg"], self.pos["lat_min"] = decimal2min(lat_decimal_deg)
        long_decimal_deg = meters2long(lkf_out[2], lat_decimal_deg, ref_long=self.ref_long)
        self.pos["long_deg"], self.pos["long_min"] = decimal2min(long_decimal_deg)
        self.vel["north"] = lkf_out[1]
        self.vel["east"] = lkf_out[3]

    def asOdom(self):
        '''
        Returns the current state estimate as an Odometry LCM object

        @return Odometry: state estimate in Odometry LCM format
        '''
        odom = Odometry()
        odom.latitude_deg = self.pos["lat_deg"]
        odom.latitude_min = self.pos["lat_min"]
        odom.longitude_deg = self.pos["long_deg"]
        odom.longitude_min = self.pos["long_min"]
        odom.bearing_deg = self.bearing_deg
        odom.speed = np.hypot(self.vel["north"], self.vel["east"])
        return odom


class SensorFusion:
    '''
    Class for filtering sensor data and outputting state estimates

    @attribute dict config: user-configured parameters found in config/filter/config.json
    @attribute Gps gps: GPS sensor
    @attribute Imu imu: IMU sensor
    @attribute Filter filter: filter used to perform sensor fusion
    @attribute StateEstimate state_estimate: current state estimate
    @attribute AsyncLCM lcm: LCM driver
    '''

    def __init__(self):
        config_path = getenv('MROVER_CONFIG')
        config_path += "/config_filter/config.json"
        with open(config_path, "r") as config:
            self.config = json.load(config)

        self.gps = Gps()
        self.imu = Imu(self.config["IMU_accel_filter_bias"], self.config["IMU_accel_threshold"])

        self.filter = None
        self.state_estimate = StateEstimate(ref_lat=self.config["RefCoords"]["lat"],
                                            ref_long=self.config["RefCoords"]["long"])

        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self._gpsCallback)
        self.lcm.subscribe("/imu_data", self._imuCallback)

    def _gpsCallback(self, channel, msg):
        new_gps = GPS.decode(msg)
        self.gps.update(new_gps)

        # Construct filter on first GPS message
        if self.filter is None and self.gps.ready():
            ref_bearing = self._getFreshBearing()
            if ref_bearing is None:
                return
            pos = self.gps.pos.asMinutes()
            # vel = self.gps.vel.absolutify(ref_bearing)
            vel = {"north": 0, "east": 0}

            self.state_estimate = StateEstimate(pos["lat_deg"], pos["lat_min"], vel["north"],
                                                pos["long_deg"], pos["long_min"], vel["east"],
                                                ref_bearing,
                                                ref_lat=self.config["RefCoords"]["lat"],
                                                ref_long=self.config["RefCoords"]["long"])
            self._constructFilter()
            self.gps.fresh = False

    def _imuCallback(self, channel, msg):
        new_imu = IMUData.decode(msg)
        self.imu.update(new_imu)
        # DEBUG
        with open("odom_accel_log.json", 'a') as out:
            out.write('{"accel_x_g": ' + str(self.imu.accel.accel_x / 9.8) + '},\n')

    def _constructFilter(self):
        '''
        Constructs filter depending on filter type
        '''
        dt = self.config['dt']

        if self.config['FilterType'] == 'LinearKalman':
            x_initial = self.state_estimate
            P_initial = self.config['P_initial']
            R = self.config['R']

            F = np.array([[1., dt, 0., 0.],
                          [0., 1., 0., 0.],
                          [0., 0., 1., dt],
                          [0., 0., 0., 1.]])

            B = np.array([[0.5*dt**2., 0.],
                         [dt, 0.],
                         [0., 0.5*dt**2.],
                         [0., dt]])

            H = np.eye(4)

            Q = QDiscreteWhiteNoise(2, dt, self.config["Q"], 2)

            self.filter = LinearKalmanFilter(4, 4, dim_u=2)
            self.filter.construct(x_initial, P_initial, F, H, Q, R, B=B)
        else:
            raise ValueError("Invalid filter type!")

    def _getFreshBearing(self):
        '''
        Returns a fresh bearing to use. Uses IMU over GPS, returns None if no fresh sensors

        @return float/None: bearing (decimal degrees East of North)
        '''
        if time.time() - self.imu.last_fresh <= self.config["IMU_fresh_timeout"]:
            return self.imu.bearing.bearing_deg
        elif time.time() - self.gps.last_fresh <= self.config["GPS_fresh_timeout"]:
            return self.gps.bearing.bearing_deg
        else:
            return None

    def _getFreshPos(self):
        '''
        Returns a fresh GPS position to use. Returns None if no fresh sensors

        @return dict/None: GPS coordinates (decimal degrees)
        '''
        if time.time() - self.gps.last_fresh <= self.config["GPS_fresh_timeout"]:
            return self.gps.pos.asDecimal()
        else:
            return None

    def _getFreshVel(self, ref_bearing):
        '''
        Returns a fresh velocity to use. Returns None if no fresh sensors

        @param float ref_bearing: reference bearing (decimal degrees East of North)
        @return dict/None: velocity North,East (m/s)
        '''
        if ref_bearing is None:
            return None

        if time.time() - self.gps.last_fresh <= self.config["GPS_fresh_timeout"]:
            return self.gps.vel.absolutify(ref_bearing)
        else:
            return None

    def _getFreshAccel(self, ref_bearing):
        '''
        Returns a fresh acceleration to use. Returns None if no fresh sensors

        @param float ref_bearing: reference bearing (decimal degrees East of North)
        @return dict/None: acceleration North,East,z (m/s^2)
        '''
        if ref_bearing is None:
            return None

        if time.time() - self.imu.last_fresh <= self.config["IMU_fresh_timeout"]:
            return self.imu.accel.absolutify(ref_bearing, self.imu.pitch_deg)
        else:
            return None

    async def run(self):
        '''
        Runs main loop for filtering data and publishing state estimates
        '''
        while True:
            if self.filter is not None:
                bearing = self._getFreshBearing()
                pos = self._getFreshPos()
                # vel = self._getFreshVel(bearing)
                vel = None
                accel = self._getFreshAccel(bearing)

                u = np.array([accel["north"], accel["east"]]) if accel is not None else None
                self.filter.predict(u=u)

                z = None
                H = None
                # vel = {"north": np.asscalar(self.filter.x[1]), "east": np.asscalar(self.filter.x[3])}
                if pos is not None:
                    pos_meters = {}
                    pos_meters["long"] = long2meters(pos["long"], pos["lat"],
                                                     ref_long=self.config["RefCoords"]["long"])
                    pos_meters["lat"] = lat2meters(pos["lat"],
                                                   ref_lat=self.config["RefCoords"]["lat"])
                    if vel is not None:
                        # If both position and velocity are available, use both
                        z = np.array([pos_meters["lat"], vel["north"], pos_meters["long"],
                                      vel["east"]])
                        H = np.eye(4)
                    else:
                        # If only position is available, zero out the velocity residual
                        z = np.array([pos_meters["lat"], 0, pos_meters["long"], 0])
                        H = np.diag([1, 0, 1, 0])
                else:
                    if vel is not None:
                        # If only velocity is availble, zero out the position residual
                        z = np.array([0, vel["north"], 0, vel["east"]])
                        H = np.diag([0, 1, 0, 1])
                self.filter.update(z, H=H)
                print(self.filter.x)

                self.state_estimate.updateFromLKF(self.filter.x)
                if bearing is not None:
                    self.state_estimate.bearing_deg = bearing

                self.gps.fresh = False
                self.imu.fresh = False

                odom = self.state_estimate.asOdom()
                self.lcm.publish('/odometry', odom.encode())
            await asyncio.sleep(self.config["UpdateRate"])


def main():
    fuser = SensorFusion()
    run_coroutines(fuser.lcm.loop(), fuser.run())


if __name__ == '__main__':
    main()
