from rover_common import aiolcm
import asyncio
from rover_common.aiohelper import run_coroutines
from enum import Enum
import can
from can.interfaces.interface import Bus

lcm_ = aiolcm.AsyncLCM()
can.rc['interface'] = 'socketcan_ctypes'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 500000
msgs = {}


async def publish_response():
    while True:
        for msg in msgs:
            if set(msg.keys()) == set([e.value for e in Odometry]):
                odom = Odometry()
                odom.latitude_deg = msg.lat_deg
                odom.latitude_min = msg.lat_min
                odom.longitude_deg = msg.lon_deg
                odom.longitude_min = msg.lon_min
                odom.bearing_deg = msg.yaw
                lcm_.publish('/odom', odom.encode())
                del msg
        print("Published response")
        await asyncio.sleep(0.1)


class Odometry(Enum):
    lat_deg = 0x50100000
    lat_min = 0x50200000
    lon_deg = 0x50300000
    lon_min = 0x50400000
    # speed_knots = 0x50500000
    # track_angle = 0x50600000
    # roll = 0x50700000
    # pitch = 0x50800000
    yaw = 0x50900000


class Listener(can.Listener):
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.msgs = {}

    def on_message_received(self, msg):
        arb_id = msg.arbitration_id
        if Odometry.had_value(arb_id):
            if not (msg.data in msgs):
                msgs[msg.data] = {}
            msgs[msg.data[4:8]][Odometry(arb_id).name] = msg.data[0:3]
        else:
            print('unknown message')


def main():
    global msgs
    bus = Bus()
    listener = Listener(bus)
    msgs = listener.msgs
    notifier = can.Notifier(bus, [listener])
    run_coroutines(lcm_.loop(), publish_response())
    notifier.stop()
