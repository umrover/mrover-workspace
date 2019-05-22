import sys
import time
import asyncio
from configparser import ConfigParser
from subprocess import Popen, PIPE
from rover_common import aiolcm
from rover_common import heartbeatlib
from rover_common.aiohelper import run_coroutines, exec_later
from rover_msgs import PiCamera, PiSettings, PiPicture
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst # noqa

lcm_ = aiolcm.AsyncLCM()

settings_path = "/home/pi/mrover-workspace/settings.ini"

settings = None
vid_process = None
pipeline = None
index = -1
secondary = False
last_ping = int(round(time.time() * 1000))
disconnected = False
reduced_quality = False
reconnection = None
taking_picture = False


def start_pipeline():
    global pipeline, vid_process, settings, secondary, reduced_quality

    height = str(settings.height)
    width = str(settings.width)

    bitrate = 300000 if reduced_quality else 1000000
    port = '5001' if secondary else '5000'

    vid_args = ["raspivid", "-t", "0", "-h", height,
                "-w", width, "-b", str(int(bitrate)),
                "-ss", str(settings.shutter_speed), "-o", "-"]
    if settings.vflip:
        vid_args.append("-vf")
        vid_args.append("-hf")

    vid_process = Popen(vid_args, stdout=PIPE)

    fd = vid_process.stdout.fileno()

    pipeline_string = ("fdsrc fd=" + str(fd) +
                       " ! video/x-h264, width=" + width +
                       ", height=" + height +
                       " ! h264parse ! rtph264pay"
                       " ! udpsink host=10.0.0.1 port=" + port)

    pipeline = Gst.parse_launch(pipeline_string)

    pipeline.set_state(Gst.State.PLAYING)

    print("Playing pipeline.", flush=True)


def stop_pipeline():
    global pipeline, vid_process

    vid_process.kill()
    vid_process = None

    pipeline.set_state(Gst.State.READY)
    pipeline = None

    print("Stopping pipeline.", flush=True)


def read_settings():
    global settings, settings_path

    config = ConfigParser()
    config.read(settings_path)

    settings = PiSettings()
    settings.shutter_speed = int(config["cam_settings"]["shutter_speed"])
    settings.vflip = config["cam_settings"].getboolean("vflip")
    settings.height = 480
    settings.width = 854


def write_settings():
    global settings, settings_path

    config = ConfigParser()
    config["cam_settings"] = {}
    config["cam_settings"]["shutter_speed"] = str(settings.shutter_speed)
    config["cam_settings"]["vflip"] = str(settings.vflip)
    config["cam_settings"]["height"] = str(settings.height)
    config["cam_settings"]["width"] = str(settings.width)

    with open(settings_path, "w") as config_file:
        config.write(config_file)


def camera_callback(channel, msg):
    global pipeline, index, secondary, taking_picture

    if taking_picture:
        return

    cam = PiCamera.decode(msg)

    if pipeline is None:
        if cam.active_index_1 == index:
            secondary = False
            start_pipeline()
        elif cam.active_index_2 == index:
            secondary = True
            start_pipeline()
    else:
        if cam.active_index_1 != index and cam.active_index_2 != index:
            secondary = False
            stop_pipeline()


def settings_callback(channel, msg):
    global settings, pipeline, index

    new_settings = PiSettings.decode(msg)
    if new_settings.pi_index != index:
        return
    settings.vflip = new_settings.vflip
    settings.shutter_speed = new_settings.shutter_speed
    print("Settings changed.", flush=True)

    stop_pipeline()
    start_pipeline()

    write_settings()


async def take_picture():
    global taking_picture
    raspistill = ('raspistill -t 1500 -o /home/pi/out_{}.jpg').format(index)
    scp = (('scp -l 2000 /home/pi/out_{}.jpg ' +
           'mrover@10.0.0.2:science-data/PiPictures/{}.jpg')
           .format(index, round(time.time() * 1000)))

    process = await asyncio.create_subprocess_shell(raspistill)
    await process.wait()
    process = await asyncio.create_subprocess_shell(scp)
    await process.wait()
    taking_picture = False


def picture_callback(channel, msg):
    global taking_picture
    data = PiPicture.decode(msg)
    if index != data.index or taking_picture:
        return
    stop_pipeline()
    taking_picture = True
    exec_later(take_picture())


def connection_state_changed(c, _):
    global disconnected, last_ping, reconnection
    if c:
        print("Connection established.", flush=True)
        disconnected = False
        reconnection = int(round(time.time() * 1000))
    else:
        last_ping = int(round(time.time() * 1000))
        disconnected = True
        reconnection = float("inf")
        print("Disconnected.", flush=True)


async def connection_monitor():
    global disconnected, last_ping, pipeline, settings, reduced_quality
    while True:
        if pipeline is not None:
            if disconnected and not reduced_quality:
                if int(round(time.time() * 1000)) - last_ping > 3000:
                    settings.height = 144
                    settings.width = 256
                    write_settings()
                    reduced_quality = True
                    stop_pipeline()
                    start_pipeline()
            if reduced_quality:
                if int(round(time.time() * 1000)) - reconnection > 5000:
                    settings.height = 480
                    settings.width = 854
                    write_settings()
                    reduced_quality = False
                    stop_pipeline()
                    start_pipeline()
        await asyncio.sleep(0.1)


def main():
    global index
    index = int(sys.argv[1])
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, index)
    Gst.init(None)

    read_settings()
    write_settings()

    lcm_.subscribe("/pi_camera", camera_callback)
    lcm_.subscribe("/pi_settings", settings_callback)
    lcm_.subscribe("/pi_picture", picture_callback)

    run_coroutines(lcm_.loop(), hb.loop(), connection_monitor())
