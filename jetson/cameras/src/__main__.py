from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Cameras
import subprocess

lcm_ = aiolcm.AsyncLCM()
pipeline = [None, None, None, None, None, None, None, None]
streaming = [False, False, False, False, False, False, False, False]
video_names = [0, 1, 2, 3, 4, 5, 6, 7]
ports = [0, 1, 3, 4, 5, 6, 7, 8]


def start_pipeline(index):
    global pipeline
    # this program assumes that the jetson has git cloned video viewer
    cmd = ("./../jetson-utils/build/aarch64/bin/video-viewer /dev/video" + str(video_names[index]) +
           " rtp://10.0.0.1:500" + str(ports[index]) + " --headless")

    pipeline[index] = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
    print("Playing camera " + str(index) + " pipeline.")


def stop_pipeline(index):
    global pipeline

    pipeline[index].kill()
    print("Stopping camera " + str(index) + " pipeline.")


def camera_callback(channel, msg):
    global pipeline
    global streaming

    camera_cmd = Cameras.decode(msg)

    requests_array = [camera_cmd.cam1, camera_cmd.cam2, camera_cmd.cam3, camera_cmd.cam4,
                      camera_cmd.cam5, camera_cmd.cam6, camera_cmd.cam7, camera_cmd.cam8]

    for i in range(len(requests_array)):
        if streaming[i] == requests_array[i]:
            continue
        streaming[i] = requests_array[i]
        if streaming[i]:
            start_pipeline(i)
        else:
            stop_pipeline(i)


def main():

    lcm_.subscribe("/cameras_cmd", camera_callback)

    run_coroutines(lcm_.loop())
