import asyncio
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import (StartTest, TestEnable, Mosfet,
                        MicroCam, RGBFrame)


lcm_ = aiolcm.AsyncLCM()

sites_busy = [False, False]

UV_LEDS_ID = "uv_leds"
AMMONIA_RGB_IDS = ["rgb_ammonia_1", "rgb_ammonia_2"]
BIRUET_RGB_IDS = ["rgb_buret_1", "rgb_buret_2"]
MICRO_CAM_IDS = ["camera_1", "camera_2"]


async def run_test(site, test):
    test_disable = TestEnable()
    test_disable.enabled = False
    test_disable.site = site
    lcm_.publish("/test_enable", test_disable.encode())

    site_index = site - 1

    mosfet = Mosfet()
    if test == "Flouresence":
        mosfet.id = UV_LEDS_ID
        mosfet.enable = True
        lcm_.publish("/mosfet", mosfet.encode())
        await asyncio.sleep(0.4)

        cam = MicroCam()
        cam.id = MICRO_CAM_IDS[site_index]
        lcm_.publish("/microcam", cam.encode())
        await asyncio.sleep(10)

        mosfet.enable = False
        lcm_.publish("/mosfet", mosfet.encode())
        await asyncio.sleep(0.4)

    elif test == "Biuret":

        await asyncio.sleep(1)
        rgb_frame = RGBFrame()
        rgb_frame.id = BIRUET_RGB_IDS[site_index]
        lcm_.publish("/rgb_frame", rgb_frame.encode())

    elif test == "Ammonia":
        await asyncio.sleep(1)
        rgb_frame = RGBFrame()
        rgb_frame.id = AMMONIA_RGB_IDS[site_index]
        lcm_.publish("/rgb_frame", rgb_frame.encode())

    test_disable.enabled = True
    lcm_.publish("/test_enable", test_disable.encode())
    sites_busy[site_index] = False


def start_test_callback(channel, msg):
    start_test = StartTest.decode(msg)
    site = start_test.site
    test = start_test.test

    site_index = start_test.site - 1
    if sites_busy[site_index]:
        return

    sites_busy[site_index] = True
    asyncio.get_event_loop().create_task(run_test(site, test))


def main():
    # look LCMSubscription.queue_capacity if messages are discarded
    lcm_.subscribe("/start_test", start_test_callback)

    run_coroutines(lcm_.loop())
