import sys
import os
import time
from configparser import ConfigParser
from subprocess import Popen, PIPE
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
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
dual_stream = False
secondary = False


def start_pipeline():
    global pipeline, vid_process, settings, dual_stream, secondary

    height = '480' if dual_stream else str(settings.height)
    width = '854' if dual_stream else str(settings.width)

    bitrate = int((settings.height * settings.width) / 0.4608)
    port = '5001' if secondary else '5000'

    vid_args = ["raspivid", "-t", "0", "-h", height,
                "-w", width, "-b", str(bitrate),
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

    print("Playing pipeline.")


def stop_pipeline():
    global pipeline, vid_process

    vid_process.kill()
    vid_process = None

    pipeline.set_state(Gst.State.READY)
    pipeline = None

    print("Stopping pipeline.")


def read_settings():
    global settings, settings_path

    config = ConfigParser()
    config.read(settings_path)

    settings = PiSettings()
    settings.shutter_speed = int(config["cam_settings"]["shutter_speed"])
    settings.vflip = config["cam_settings"].getboolean("vflip")
    settings.height = int(config["cam_settings"]["height"])
    settings.width = int(config["cam_settings"]["width"])


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
    global pipeline, index, dual_stream, secondary

    cam = PiCamera.decode(msg)
    dual_stream = cam.dual_stream

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
    settings = new_settings
    print("Settings changed.")

    stop_pipeline()
    start_pipeline()

    write_settings()


def picture_callback(channel, msg):
    data = PiPicture.decode(msg)
    if index != data.index:
        return
    stop_pipeline()
    os.system('raspistill -t 1500 -o /home/pi/out_{}.jpg'.format(index))
    start_pipeline()
    os.system('scp -l 2000 /home/pi/out_{}.jpg ' +
              'mrover@10.0.0.2:science-data/PiPictures/{}.jpg'
              .format(index, round(time.time() * 1000)))


def main():
    global index
    index = int(sys.argv[1])
    Gst.init(None)

    read_settings()

    lcm_.subscribe("/pi_camera", camera_callback)
    lcm_.subscribe("/pi_settings", settings_callback)
    lcm_.subscribe("/pi_picture", picture_callback)

    run_coroutines(lcm_.loop())
