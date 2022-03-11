from rover_msgs import Cameras
import lcm

import sys
sys.path.insert(0, "/usr/lib/python3.8/dist-packages")
import jetson.utils

__lcm: lcm.LCM
__pipelines = [None] * 8
__devices_enabled = 0
__oldest_pipeline = -1
__oldest_port = -1
__most_recent_pipeline = -1
__most_recent_port = -1


ARGUMENTS = ['--headless']

class Pipeline:
    def __init__(self, index):
        self.video_source = jetson.utils.videoSource(f"/dev/video{index}", argv=ARGUMENTS)
        self.video_output = None

    def update(self):
        image = self.video_source.Capture()
        self.video_output.Render(image)

    def is_open(self):
        return True


def start_pipeline(index, port):
    global __pipelines

    try:
        self.video_output = jetson.utils.videoOutput(f"rtp://10.0.0.1:500{port}", argv=ARGUMENTS)
        __pipelines[index] = Pipeline(index)
        print(f"Playing camera {index} __pipelines.")
    except Exception as e:
        pass



def stop_pipeline(index):
    global __pipelines

    __pipelines[index] = None
    print(f"Stopping camera {index} __pipelines.")


def start_pipeline_and_update_globals(pipeline):
    global __devices_enabled, __oldest_pipeline, __oldest_port, __most_recent_pipeline, __most_recent_port
    if __devices == 0:
        start_pipeline(pipeline, 0)
        __oldest_port, __most_recent_port = 0, 0
        __oldest_pipeline, __most_recent_pipeline = pipeline, pipeline
        __devices_enabled += 1
    elif __devices_enabled == 1:
        if __most_recent_port == 0:
            start_pipeline(pipeline, 1)
            __oldest_port, __most_recent_port = 0, 1
            __oldest_pipeline, __most_recent_pipeline = __most_recent_pipeline, pipeline
        elif __most_recent_port == 1:
            start_pipeline(pipeline, 0)
            __oldest_port, __most_recent_port = 1, 0
        __devices_enabled += 1
    elif __devices_enabled == 2:
        stop_pipeline(__oldest_pipeline)
        start_pipeline(pipeline, __oldest_port)
        __oldest_port, __most_recent_port = __most_recent_port, __oldest_port
        __oldest_pipeline, __most_recent_pipeline = __most_recent_pipeline, __oldest_pipeline


def stop_pipeline_and_update_globals(pipeline):
    global __devices_enabled, __oldest_pipeline, __oldest_port, __most_recent_pipeline, __most_recent_port
    stop_pipeline(pipeline)
    if __devices_enabled == 1:
        __oldest_port, __most_recent_port = -1, -1
        __oldest_pipeline, __most_recent_pipeline = -1, -1
    elif __devices_enabled == 2:
        if pipeline == __oldest_pipeline:
            __oldest_pipeline = __most_recent_pipeline
            __oldest_port = __most_recent_port
        elif pipeline == __most_recent_pipeline:
            __most_recent_pipeline = __oldest_pipeline
            __most_recent_port = __oldest_port
    __devices_enabled -= 1


def camera_callback(channel, msg):
    global __pipelines, __is_streaming

    camera_cmd = Cameras.decode(msg)

    enabled_cameras = [camera_cmd.cam1, camera_cmd.cam2, camera_cmd.cam3, camera_cmd.cam4,
                       camera_cmd.cam5, camera_cmd.cam6, camera_cmd.cam7, camera_cmd.cam8]

    # print(enabled_cameras)

    for i, is_enabled in enumerate(enabled_cameras):
        current_is_enabled = __pipelines[i] is not None
        if is_enabled == current_is_enabled:
            continue
        if is_enabled:
            start_pipeline_and_update_globals(i)
        else:
            stop_pipeline_and_update_globals(i)


def main():
    global __pipelines, __lcm

    __lcm = lcm.LCM()
    __lcm.subscribe("/cameras_cmd", camera_callback)
    while True:
        while __lcm.handle_timeout(0):
            pass
        for i, pipeline in enumerate(__pipelines):
            if pipeline is not None:
                if pipeline.is_open():
                    pipeline.update()
                else:
                    stop_pipeline_and_update_globals(i)
                    print(f'Closing {i} becuase it stopped streaming!')


if __name__ == "__main__":
    main()
