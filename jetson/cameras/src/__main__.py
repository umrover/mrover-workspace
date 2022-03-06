from rover_msgs import Cameras
import lcm

import sys
sys.path.insert(0, "/usr/lib/python3.8/dist-packages")
import jetson.utils

__lcm: lcm.LCM
__pipelines = [None] * 8

ARGUMENTS = ['--headless']

class Pipeline:
    def __init__(self, index):
        self.video_source = jetson.utils.videoSource(f"/dev/video{index}", argv=ARGUMENTS)
        self.video_output = jetson.utils.videoOutput(f"rtp://10.0.0.1:500{index + 3}", argv=ARGUMENTS)

    def update(self):
        image = self.video_source.Capture()
        self.video_output.Render(image)

    def is_open(self):
        return True


def start_pipelines(index):
    global __pipelines

    try:
        __pipelines[index] = Pipeline(index)
        print(f"Playing camera {index} __pipelines.")
    except Exception as e:
        pass



def stop_pipelines(index):
    global __pipelines

    __pipelines[index] = None
    print(f"Stopping camera {index} __pipelines.")


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
            start_pipelines(i)
        else:
            stop_pipelines(i)


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
                    __pipelines[i] = None
                    print(f'Closing {i} becuase it stopped streaming!')


if __name__ == "__main__":
    main()
