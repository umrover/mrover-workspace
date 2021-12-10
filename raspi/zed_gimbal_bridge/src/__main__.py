# import lcm
import asyncio
import math
import moteus
import moteus_pi3hat
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ZedGimbalPosition, ZedGimbalCmd

lcm_ = aiolcm.AsyncLCM()

f = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={1: [1]},
)

c = moteus.Controller(transport=f, id=1)
desired_position_revolutions = 0


def zed_gimbal_position_callback(channel, msg):
    zed_struct = ZedGimbalCmd.decode(msg)
    desired_position_degrees = zed_struct.angle
    global desired_position_revolutions
    desired_position_revolutions = desired_position_degrees / 360.0
    c.make_position(position=math.nan, velocity=0.2, maximum_torque=0.3,
                    stop_position=desired_position_revolutions, watchdog_timeout=10, query=True)


async def publish_zed_gimbal_position():
    while (True):
        global desired_position_revolutions
        state = await c.set_position(position=math.nan, velocity=0.2, maximum_torque=0.3,
                                     stop_position=desired_position_revolutions, watchdog_timeout=70, query=True)

        position_revolutions = state.values[moteus.Register.POSITION]
        position_degrees = 360.0 * position_revolutions
        zed_struct = ZedGimbalPosition()
        zed_struct.angle = position_degrees
        lcm_.publish('/zed_gimbal_data', zed_struct.encode())

        await asyncio.sleep(0.5)


def main():

    lcm_.subscribe("/zed_gimbal_cmd", zed_gimbal_position_callback)

    run_coroutines(lcm_.loop(), publish_zed_gimbal_position())


if __name__ == "__main__":
    main()
