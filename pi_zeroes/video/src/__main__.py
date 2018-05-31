import sys
from configparser import ConfigParser
from subprocess import Popen, PIPE
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import PiCamera, PiSettings
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst # noqa

lcm_ = aiolcm.AsyncLCM()

settings_path = "/home/pi/mrover-workspace/settings.ini"

settings = None
vid_process = None
pipeline = None
index = -1


def start_pipeline():
    global pipeline
    global vid_process
    global settings

    height = str(settings.height)
    width = str(settings.width)
    bitrate = int((settings.height * settings.width) / 0.4608)

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
                       " ! udpsink host=10.0.0.1 port=5000")

    pipeline = Gst.parse_launch(pipeline_string)

    pipeline.set_state(Gst.State.PLAYING)

    print("Playing pipeline.")


def stop_pipeline():
    global pipeline
    global vid_process

    vid_process.kill()
    vid_process = None

    pipeline.set_state(Gst.State.READY)
    pipeline = None

    print("Stopping pipeline.")


def read_settings():
    global settings
    global settings_path

    config = ConfigParser()
    config.read(settings_path)

    settings = PiSettings()
    settings.shutter_speed = int(config["cam_settings"]["shutter_speed"])
    settings.vflip = config["cam_settings"].getboolean("vflip")
    settings.height = int(config["cam_settings"]["height"])
    settings.width = int(config["cam_settings"]["width"])


def write_settings():
    global settings
    global settings_path

    config = ConfigParser()
    config["cam_settings"] = {}
    config["cam_settings"]["shutter_speed"] = str(settings.shutter_speed)
    config["cam_settings"]["vflip"] = str(settings.vflip)
    config["cam_settings"]["height"] = str(settings.height)
    config["cam_settings"]["width"] = str(settings.width)

    with open(settings_path, "w") as config_file:
        config.write(config_file)


def camera_callback(channel, msg):
    global pipeline
    global index

    cam_info = PiCamera.decode(msg)

    if cam_info.active_index == index and pipeline is None:
        start_pipeline()
    elif cam_info.active_index != index and pipeline is not None:
        stop_pipeline()


def settings_callback(channel, msg):
    global settings
    global pipeline

    if pipeline is None:
        return

    settings = PiSettings.decode(msg)
    print("Settings changed.")

    stop_pipeline()
    start_pipeline()

    write_settings()


def main():
    global index
    index = int(sys.argv[1])
    Gst.init(None)

    read_settings()

    lcm_.subscribe("/pi_camera", camera_callback)
    lcm_.subscribe("/pi_settings", settings_callback)

    run_coroutines(lcm_.loop())
