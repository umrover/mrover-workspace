from rover_msgs import Cameras
import lcm

import sys
sys.path.insert(0, "/usr/lib/python3.6/dist-packages")  # 3.6 vs 3.8
import jetson.utils

__lcm: lcm.LCM
__pipelines = [None] * 2

ARGUMENTS = ['--headless', '--bitrate=300000', '--width=256', '--height=144']  # Change to desired bitrate e.g. 2000000
remote_ip = ["10.0.0.1:5000", "10.0.0.1:5001"]


class Pipeline:
    def __init__(self, port):
        self.video_source = None
        self.video_output = jetson.utils.videoOutput(f"rtp://{remote_ip[port]}", argv=ARGUMENTS)
        self.device_number = -1
        self.port = port

    def update(self):
        image = self.video_source.Capture()
        self.video_output.Render(image)

    def is_open(self):
        return True

    def is_device_number(self):
        return self.device_number

    def port(self):
        return self.port

    def update_device_number(self, index):
        if index != -1:
            self.video_output = jetson.utils.videoOutput(f"rtp://{remote_ip[self.port]}", argv=ARGUMENTS)
            self.video_source = jetson.utils.videoSource(f"/dev/video{index}", argv=ARGUMENTS)
        else:
            self.video_source = None
        self.device_number = index

    def is_currently_streaming(self):
        return self.device_number != -1


def start_pipeline(index, port):
    global __pipelines

    try:
        __pipelines[port].update_device_number(index)
        print(f"Playing camera {index} __pipelines on port 500{port}.")
    except Exception:
        pass


def stop_pipeline(port):
    global __pipelines

    print(f"Stopping camera {__pipelines[port].device_number} on port 500{port}.")
    __pipelines[port].update_device_number(-1)


def camera_callback(channel, msg):
    global __pipelines

    camera_cmd = Cameras.decode(msg)

    port_devices = [camera_cmd.port_0, camera_cmd.port_1]

    for port_number, requested_port_device in enumerate(port_devices):
        if __pipelines[port_number].is_device_number() == requested_port_device:
            continue
        if requested_port_device == -1:
            stop_pipeline(port_number)
        else:
            if __pipelines[port_number].is_currently_streaming():
                stop_pipeline(port_number)
            start_pipeline(requested_port_device, port_number)


def main():
    global __pipelines, __lcm

    __pipelines = [Pipeline(0), Pipeline(1)]

    __lcm = lcm.LCM()
    __lcm.subscribe("/cameras_cmd", camera_callback)
    while True:
        while __lcm.handle_timeout(0):
            pass
        for port_number, pipeline in enumerate(__pipelines):
            if pipeline.is_currently_streaming():
                if pipeline.is_open():
                    # This is currently always True
                    pipeline.update()
                else:
                    stop_pipeline(port_number)
                    print(f'Closing {port_number} becuase it stopped streaming!')


if __name__ == "__main__":
    main()
