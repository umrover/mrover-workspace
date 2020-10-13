import sys


UNDEFINED = 0#-999

class raw_imu:
    def __init__(self):
        # global UNDEFINED
        self.mag_bearing = UNDEFINED # TODO Need to read in mag_bearing correctly.
        self.gx = UNDEFINED
        self.gy = UNDEFINED
        self.gz = UNDEFINED
        self.accx = UNDEFINED
        self.accy = UNDEFINED
        self.accz = UNDEFINED
        self.time_of_IMU = time()

    # def updateIMU(mbearing,GX,GY,GZ,AX,AY,AZ):
    def updateIMU(message):
        self.gx = message.gyrox
        self.gy = message.gyroy
        self.gz = message.gyroz
        self.accx = message.accx
        self.accy = message.accy
        self.accz = message.accz
        self.mag_bearing = message.mag
        self.time_of_IMU = time()



class raw_gps:
    def __init__(self):
        # global UNDEFINED
        self.lat_deg = UNDEFINED
        self.lat_min = UNDEFINED
        self.long_deg = UNDEFINED
        self.long_min = UNDEFINED
        self.track_angle = UNDEFINED
        self.ground_speed = UNDEFINED
        self.time_of_GPS = time()

    # def updateGPS(latd, latm, longd, longm, ttheta, gspeed):
    def updateGPS(message):
        self.lat_deg = message.latitude_deg
        self.lat_min = message.latitude_min
        self.long_deg = message.longitude_deg
        self.long_min = message.longitude_min
        self.track_angle = message.bearing_deg
        self.ground_speed = message.groundspeed
        self.time_of_GPS = time()

class raw_wheelenc:
    def __init__(self):
        # global UNDEFINED
        self.rear_l = UNDEFINED
        self.rear_r = UNDEFINED
        self.front_r = UNDEFINED
        self.front_r = UNDEFINED
        self.time_of_ENC = time()

    # def updateENC(rl, rr, fl, fr):
    def updateENC(message):
        self.rear_l = message.rearleft  # Names not established: todo
        self.rear_r = message.rearright
        self.front_l = message.frontleft
        self.front_r = message.frontright
        self.time_of_ENC = time()

