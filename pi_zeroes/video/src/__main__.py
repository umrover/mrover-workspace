import sys
from subprocess import Popen, PIPE
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import PiCamera
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst # noqa

lcm_ = aiolcm.AsyncLCM()

pipeline = None
pipeline_state = Gst.State.READY
index = -1


def init_pipeline():
    global pipeline

    vid_process = Popen(["raspivid", "-t", "0", "-h", "720", "-w", "1280",
                         "-hf", "-b", "2000000", "-o", "-"], stdout=PIPE)

    fd = vid_process.stdout.fileno()

    pipeline_string = ("fdsrc fd=" + str(fd) +
                       " ! video/x-h264, width=1280, height=720"
                       " ! h264parse ! rtph264pay"
                       " ! udpsink host=10.0.0.1 port=5000")

    Gst.init(None)
    pipeline = Gst.parse_launch(pipeline_string)

    print("Initialized gstreamer pipeline.")


def camera_callback(channel, msg):
    global pipeline
    global pipeline_state
    global index

    cam_info = PiCamera.decode(msg)

    if cam_info.active_index == index and pipeline_state != Gst.State.PLAYING:
        pipeline_state = Gst.State.PLAYING
        pipeline.set_state(pipeline_state)
        print("Playing pipeline.")
    elif cam_info.active_index != index and pipeline_state != Gst.State.READY:
        pipeline_state = Gst.State.READY
        pipeline.set_state(pipeline_state)
        print("Pausing pipeline.")


def main():
    global index
    index = int(sys.argv[1])

    init_pipeline()

    lcm_.subscribe("/pi_camera", camera_callback)

    run_coroutines(lcm_.loop())
