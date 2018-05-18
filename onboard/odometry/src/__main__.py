import struct
import usb
import time
import sys
import traceback
from rover_common import aiolcm
from rover_msgs import Odometry

from .android_usb_comm import Android


GS3_DEVICE_IDS = (0x18d1, 0x4ee7, 0x2d01)

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
        *GS3_DEVICE_IDS,
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
                print('connected to Samsung GS3')
                msg = Odometry()
                while True:
                    raw_data = android.read()
                    if raw_data:
                        frame = AndroidFrame(raw_data)
                        msg.latitude_deg = int(frame.lat_deg)
                        msg.latitude_min = frame.lat_min
                        msg.longitude_deg = int(frame.lon_deg)
                        msg.longitude_min = frame.lon_min
                        msg.bearing_deg = frame.azimuth_deg
                        msg.num_satellites = frame.num_sats
                        lcm_.publish('/odom', msg.encode())
                    else:
                        print('read timed out, retrying the connection')
                        break
        except usb.core.USBError:
            traceback.print_exception(*sys.exc_info())
            print('USBError occured, retrying...')
