import asyncio
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (StartTest, TestEnable, Mosfet,
                        MicroCam, RGBFrame, Servo)


lcm_ = aiolcm.AsyncLCM()

sites_busy = [False, False]

UV_LEDS_ID = "uv_leds"
AMMONIA_SERVO_IDS = ["servo_1", "servo_2"]
AMMONIA_RGB_IDS = ["rgb_ammonia_1", "rgb_ammonia_2"]
BIRUET_RGB_IDS = ["rgb_buret_1", "rgb_buret_2"]
MICRO_CAM_IDS = ["camera_1", "camera_2"]

AMMONIA_SERVO_INACTIVE = 0
AMMONIA_SERVO_ACTIVE = 45


async def run_test(site, test):
    test_disable = TestEnable()
    test_disable.enabled = False
    test_disable.site = site
    lcm_.publish("/test_enable", test_disable.encode())

    mosfet = Mosfet()
    if test == "Flouresence":
        mosfet.id = UV_LEDS_ID
        mosfet.enable = True
        lcm_.publish("/mosfet", mosfet.encode())
        await asyncio.sleep(0.4)

        cam = MicroCam()
        cam.id = MICRO_CAM_IDS[site]
        lcm_.publish("/micro_cam", cam.encode())
        await asyncio.sleep(3)

        mosfet.enable = False
        lcm_.publish("/mosfet", mosfet.encode())
        await asyncio.sleep(0.4)

    elif test == "Biuret":

        await asyncio.sleep(1)
        rgb_frame = RGBFrame()
        rgb_frame.id = BIRUET_RGB_IDS[site]
        lcm_.publish("/rgb_frame", rgb_frame.encode())

    elif test == "Ammonia":
        rgb_frame = RGBFrame()
        rgb_frame.id = AMMONIA_RGB_IDS[site]
        lcm_.publish("/rgb_frame", rgb_frame.encode())
        await asyncio.sleep(1.2)

        servo = Servo()
        servo.id = AMMONIA_SERVO_IDS[site]
        servo.degrees = AMMONIA_SERVO_ACTIVE
        lcm_.publish("/servo", servo.encode())
        await asyncio.sleep(2.5)

        servo.degrees = AMMONIA_SERVO_INACTIVE
        lcm_.publish("/servo", servo.encode())
        await asyncio.sleep(1.6)

        lcm_.publish("/rgb_frame", rgb_frame.encode())
        await asyncio.sleep(0.6)

    test_disable.enabled = True
    lcm_.publish("/test_enable", test_disable.encode())
    sites_busy[site] = False


def start_test_callback(channel, msg):
    start_test = StartTest.decode(msg)

    if sites_busy[start_test.site]:
        return

    sites_busy[start_test.site] = True
    asyncio.get_event_loop().create_task(run_test(start_test.site,
                                                  start_test.test))


def main():
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/start_test", start_test_callback)

    run_coroutines(lcm_.loop())
