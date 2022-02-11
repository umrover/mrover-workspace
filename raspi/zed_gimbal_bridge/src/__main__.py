import asyncio
import math  # needed for math.nan
import moteus
import moteus_pi3hat
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ZedGimbalPosition

lcm_ = aiolcm.AsyncLCM()

# assume connection to JC1
f = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={1: [1]},
)

c = moteus.Controller(transport=f, id=1)
desired_rev = 0


def restrict_angle(input_angle):
    lower_bound = -180
    upper_bound = 180
    if input_angle > upper_bound:
        return upper_bound
    elif input_angle < lower_bound:
        return lower_bound
    return input_angle


def zed_gimbal_position_callback(channel, msg):
    zed_struct = ZedGimbalPosition.decode(msg)
    degrees = zed_struct.angle
    degrees = restrict_angle(degrees)
    global desired_rev
    desired_rev = degrees / 360.0


async def publish_zed_gimbal_position():
    while (True):
        global desired_rev

        state = await c.set_position(position=math.nan, velocity=0.2, maximum_torque=0.3,
                                     stop_position=desired_rev, watchdog_timeout=70, query=True)

        fault_value = state.values[moteus.Register.FAULT]
        if fault_value != 0:
            print("Error #" + str(fault_value))
            await c.set_stop()
            await asyncio.sleep(0.5)
            continue

        rev = state.values[moteus.Register.POSITION]
        degrees = 360.0 * rev
        degrees = restrict_angle(degrees)

        zed_struct = ZedGimbalPosition()
        zed_struct.angle = degrees
        lcm_.publish('/zed_gimbal_data', zed_struct.encode())

        await asyncio.sleep(0.5)


def main():
    lcm_.subscribe("/zed_gimbal_cmd", zed_gimbal_position_callback)
    run_coroutines(lcm_.loop(), publish_zed_gimbal_position())


if __name__ == "__main__":
    main()
