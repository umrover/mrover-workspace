import asyncio
import random
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Odometry


lcm_ = aiolcm.AsyncLCM()


def connection_state_changed(c):
    if c:
        print("Connection established.")
    else:
        print("Disconnected.")

def joystick_callback(channel, msg):
    input_data = Joystick.decode(msg)
    print("Recieving {} bytes from {}".format(len(msg), channel))


async def transmit_fake_odometry():
    while True:
        new_odom = Odometry()
        new_odom.latitude_deg = random.uniform(42, 43)
        new_odom.longitude_deg = random.uniform(-84, -82)
        new_odom.bearing_deg = random.uniform(0, 359)
        lcm_.publish('/odom', new_odom.encode())

        print("Published new odometry")
        await asyncio.sleep(0.5)


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed)
    #look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/joystick", joystick_callback)
    run_coroutines(hb.loop(), transmit_fake_odometry())
