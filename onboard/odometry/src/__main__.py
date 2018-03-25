from rover_common import aiolcm
import asyncio
from enum import Enum
import os
from rover_common.aiohelper import run_coroutines, wait_for
from rover_common import aiocan


lcm_ = aiolcm.AsyncLCM()
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


class Listener:
    def __init__(self, bus):
        super().__init__()
        self.bus = bus

    def __call__(self, msg):
        if msg.arb_id in Odometry:
            msgs[msg.data[4:8]][Odometry(msg.arb_id).name] = msg.data[0:3]


async def setup_bus(bus):
    l = Listener(bus)
    await bus.subscribe(Odometry.lat_deg.value, l, extended=True)
    await bus.subscribe(Odometry.lat_min.value, l, extended=True)
    await bus.subscribe(Odometry.lon_deg.value, l, extended=True)
    await bus.subscribe(Odometry.lon_min.value, l, extended=True)
    await bus.subscribe(Odometry.yaw.value, l, extended=True)


def main():
    global msgs
    CHANNEL = os.environ.get('MROVER_TALON_CAN_IFACE', 'vcan0')
    bus = aiocan.AsyncBCM(CHANNEL)
    wait_for(setup_bus(bus))
    run_coroutines(lcm_.loop(), publish_response())
