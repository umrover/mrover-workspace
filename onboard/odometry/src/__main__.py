import serial
import struct
import os
import threading
import pynmea2
from rover_common import aiolcm, frame_serial
from rover_msgs import Odometry


lcm_ = aiolcm.AsyncLCM()
odom_port = os.environ.get('MROVER_ODOM_PORT', '/dev/ttyUSB0')
gps_port = os.environ.get('MROVER_GPS_PORT', '/dev/ttyUSB1')


mutex = threading.Lock()
condition = threading.Condition(mutex)
start_condition = threading.Barrier(3)

imu_ready = False
gps_ready = False
sat_count_ready = False
changed = False
gps_valid = False
msg = Odometry()
num_satellites = 0


class OdomFrame:
    def __init__(self, buf):
        data = struct.unpack('<fffififi??', buf)
        self.roll = data[0]
        self.pitch = data[1]
        self.bearing = data[2]
        self.lat_deg = data[3]
        self.lat_min = data[4]
        self.lon_deg = data[5]
        self.lon_min = data[6]
        self.num_sats = data[7]
        self.gps_read = data[8]
        self.imu_read = data[9]


def read_odom_board():
    global imu_ready
    global mutex
    global condition
    global start_condition
    global changed
    try:
        odom_ser = serial.Serial(odom_port, timeout=1)
        odom_ser.baudrate = 115200

        r = frame_serial.Reader()
        counter = 0

        start_condition.wait()
    except:
        start_condition.abort()
        raise

    while True:
        c = odom_ser.read()
        if r.feed(c):
            try:
                frame = OdomFrame(b''.join(r.buffer))
                if not frame.imu_read:
                    print(
                        '{}: Could not read a complete'
                        'IMU frame'.format(counter))
                with mutex:
                    msg.bearing_deg = frame.bearing
                    imu_ready = True
                    changed = True
                    condition.notify()
            except struct.error:
                print("Incomplete data, only {} bytes".format(
                    len(r.buffer)))


def read_gps():
    global gps_ready
    global sat_count_ready
    global gps_valid
    global changed
    global mutex
    global condition
    global start_condition
    global num_satellites
    try:
        gps_ser = serial.Serial(gps_port, timeout=1)
        gps_ser.baudrate = 4800

        start_condition.wait()
    except:
        start_condition.abort()
        raise

    while True:
        sentence = gps_ser.readline().decode('iso-8859-1')
        try:
            nmea = pynmea2.parse(sentence)
        except pynmea2.ParseError as e:
            print("parse error: {}".format(e))
            continue
        if isinstance(nmea, pynmea2.RMC):
            # TODO rethink this "not publishing" when losing a fix
            with mutex:
                if nmea.status == 'A':
                    gps_valid = True
                    msg.latitude_deg = int(nmea.latitude)
                    msg.latitude_min = nmea.latitude_minutes
                    if msg.latitude_deg < 0:
                        msg.latitude_min *= -1
                    msg.longitude_deg = int(nmea.longitude)
                    msg.longitude_min = nmea.longitude_minutes
                    if msg.longitude_deg < 0:
                        msg.longitude_min *= -1
                    gps_ready = True
                    changed = True
                    condition.notify()
                elif nmea.status == 'V':
                    gps_valid = False
        elif isinstance(nmea, pynmea2.GSV):
            with mutex:
                msg.num_satellites = int(nmea.num_sv_in_view)
                sat_count_ready = True
                changed = True
                condition.notify()


def main():
    global mutex
    global condition
    global msg
    global imu_ready
    global gps_ready
    global sat_count_ready
    global changed
    global gps_valid
    global start_condition
    global num_satellites

    # start threads
    t1 = threading.Thread(target=read_odom_board)
    t2 = threading.Thread(target=read_gps)
    t1.start()
    t2.start()

    start_condition.wait()

    with mutex:
        condition.wait_for(lambda: imu_ready and gps_ready and sat_count_ready)

    while True:
        with mutex:
            condition.wait_for(lambda: changed)
            if gps_valid:
                lcm_.publish('/odom', msg.encode())
            changed = False
