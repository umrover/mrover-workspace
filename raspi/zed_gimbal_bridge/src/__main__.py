# import lcm
import asyncio
import math
import moteus
import moteus_pi3hat
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ZedGimbalPosition

lcm_ = aiolcm.AsyncLCM()

f = moteus_pi3hat.Pi3HatRouter(
    servo_bus_map={...},
)

c = moteus.Controller(transport=f, id=1)


def zed_gimbal_position_callback(channel, msg):
    desired_position_degrees = ZedGimbalPosition.decode(msg)
    desired_position_revolutions = desired_position_degrees / 360.0
    c.make_position(position=math.nan, velocity=0.2, maximum_torque=0.3,
                    stop_position=desired_position_revolutions, watchdog_timeout=10, query=True)


async def publish_zed_gimbal_position():

    while(True):
        state = await c.set_position(position=math.nan, query=True)

        position_revolutions = state.values[moteus.Register.POSITION]
        position_degrees = 360.0 * position_revolutions

        lcm_.publish('/zed_gimbal_data', position_degrees.encode())

        await asyncio.sleep(0.5)


async def main():

    lcm_.subscribe("/zed_gimbal_data", zed_gimbal_position_callback)

    run_coroutines(lcm_.loop(), publish_zed_gimbal_position())


if __name__ == "__main__":
    main()
