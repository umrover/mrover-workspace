import struct
import usb
import time
import sys
import traceback
from rover_common import aiolcm
from rover_msgs import IMU, GPS

from .android_usb_comm import Android


PHONE_DEVICE_IDS = (0x1004, 0x18d1, 0x633e, 0x2d01)

lcm_ = aiolcm.AsyncLCM()


class AndroidFrame:
    decode_fmt = struct.Struct('>iififfii')

    def __init__(self, buf):
        assert len(buf) == AndroidFrame.decode_fmt.size
        vals = AndroidFrame.decode_fmt.unpack(buf)
        (padding, self.lat_deg, self.lat_min, self.lon_deg, self.lon_min,
         self.azimuth_deg, self.num_sats, self.valid) = vals
        self.valid = True if self.valid == 1 else False


def get_android():
    return Android(
        *PHONE_DEVICE_IDS,
        manufacturer='MRover',
        model='Onboard',
        description='OnboardComputer',
        version=1,
        uri='https://github.com/umrover/mrover-workspace',
        serial='S0001')


def main():
    while True:
        try:
            time.sleep(1)
            with get_android() as android:
                print('connected to Sensor Package')

                while True:
                    raw_data = android.read()
                    if raw_data:
                        frame = AndroidFrame(raw_data)

                        imu_msg = IMU()
                        imu_msg.bearing = frame.azimuth_deg
                        lcm_.publish('/imu', imu_msg.encode())

                        gps_msg = GPS()
                        gps_msg.latitude_deg = int(frame.lat_deg)
                        gps_msg.latitude_min = frame.lat_min
                        gps_msg.longitude_deg = int(frame.lon_deg)
                        gps_msg.longitude_min = frame.lon_min
                        gps_msg.bearing_deg = frame.azimuth_deg
                        gps_msg.speed = 0
                        lcm_.publish('/gps', gps_msg.encode())
                    else:
                        print('read timed out, retrying the connection')
                        break
        except usb.core.USBError:
            traceback.print_exception(*sys.exc_info())
            print('USBError occured, retrying...')
