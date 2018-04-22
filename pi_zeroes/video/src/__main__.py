import sys
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst # noqa
from rover_common import aiolcm # noqa
from rover_common.aiohelper import run_coroutines # noqa
from rover_msgs import PiCamera # noqa

lcm_ = aiolcm.AsyncLCM()

pipeline = None
pipeline_state = Gst.State.READY
pipeline_string = ("v4l2src device=/dev/video0"
                   " ! image/jpeg, width=1280, height=720, framerate=30/1"
                   " ! jpegdec ! videoconvert ! queue ! x264enc ! queue"
                   " ! rtph264pay ! udpsink host=192.168.0.98 port=5000")
index = -1


def init_pipeline():
    global pipeline
    global pipeline_string
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
