 from rover_msgs import Cameras, Mission
import lcm
from enum import Enum

import sys
sys.path.insert(0, "/usr/lib/python3.6/dist-packages")  # 3.6 vs 3.8
import jetson.utils  # noqa

__lcm: lcm.LCM
__pipelines = [None] * 4

ARGUMENTS_144 = ['--headless', '--bitrate=300000', '--width=256', '--height=144']
ARGUMENTS_360 = ['--headless', '--bitrate=800000', '--width=480', '--height=360']
ARGUMENTS_720 = ['--headless', '--bitrate=1800000', '--width=1280', '--height=720']

# 10.0.0.1 represents the ip of the main base station laptop
# 10.0.0.2 represents the ip of the secondary science laptop
auton_ips = ["10.0.0.1:5000", "10.0.0.1:5001", "N/A", "N/A"]
erd_ips = ["10.0.0.1:5000", "10.0.0.1:5001", "N/A", "N/A"]
es_ips = ["10.0.0.1:5000", "10.0.0.1:5001", "N/A", "N/A"]
science_ips = ["10.0.0.1:5000", "10.0.0.1:5001", "10.0.0.2:5000", "10.0.0.2:5001"]


class MissionNames(Enum):
    AUTON = 0
    ERD = 1
    ES = 2
    SCIENCE = 3


mission_map = {
    "AUTON": [MissionNames.AUTON, auton_ips, ARGUMENTS_360],
    "ERD": [MissionNames.ERD, erd_ips, ARGUMENTS_360],
    "ES": [MissionNames.ES, es_ips, ARGUMENTS_720],
    "SCIENCE": [MissionNames.SCIENCE, science_ips, ARGUMENTS_360]
}

# default to science
current_mission, current_mission_ip, ARGUMENTS = mission_map["SCIENCE"]

video_sources = [None] * 10


class Pipeline:
    def __init__(self, port):
        global current_mission_ip, current_mission
        self.current_ips = current_mission_ip
        self.video_source = None
        self.video_output = jetson.utils.videoOutput(f"rtp://{self.current_ips[port]}", argv=ARGUMENTS)
        self.device_number = -1
        self.port = port

    def update_video_output(self):
        global current_mission_ip, current_mission
        self.current_ips = current_mission_ip
        self.video_output = jetson.utils.videoOutput(f"rtp://{self.current_ips[self.port]}", argv=ARGUMENTS)

    def capture_and_render_image(self):
        try:
            image = self.video_source.Capture()
            self.video_output.Render(image)
        except Exception:
            print(f"Camera capture {self.device_number} on {self.current_ips[self.port]} failed. Stopping stream.")
            failed_device_number = self.device_number
            self.device_number = -1
            if pipeline_device_is_unique(self.port, failed_device_number):
                close_video_source(failed_device_number)

    def is_open(self):
        return True

    def get_device_number(self):
        return self.device_number

    def port(self):
        return self.port

    def update_device_number(self, index):
        self.device_number = index
        if index != -1:
            self.video_source = video_sources[index]
            if self.video_source is not None:
                self.video_output = jetson.utils.videoOutput(f"rtp://{self.current_ips[self.port]}", argv=ARGUMENTS)
            else:
                print(f"Unable to play camera {index} on {self.current_ips[self.port]}.")
                self.device_number = -1
        else:
            self.video_source = None

    def is_currently_streaming(self):
        return self.device_number != -1


def start_pipeline(index, port):
    global __pipelines, current_mission_ip
    try:
        __pipelines[port].update_device_number(index)
        print(f"Playing camera {index} on {current_mission_ip[port]}.")
    except Exception:
        pass


def close_video_source(index):
    global video_sources
    video_sources[index] = None


def create_video_source(index):
    global video_sources
    if index == -1:
        return
    if video_sources[index] is not None:
        return
    try:
        video_sources[index] = jetson.utils.videoSource(f"/dev/video{index}", argv=ARGUMENTS)
    except Exception:
        pass


def pipeline_device_is_unique(excluded_pipeline, device_number):
    # This function checks whether excluded_pipeline is the only pipeline streaming device device_number
    global __pipelines
    # check if any of the other pipelines are using the current device
    for pipeline_number, pipeline in enumerate(__pipelines):
        if pipeline_number == excluded_pipeline:
            continue
        if pipeline.get_device_number() == device_number:
            return False
    return True


def mission_callback(channel, msg):
    global __pipelines, current_mission, current_mission_ip, ARGUMENTS
    mission_name = Mission.decode(msg).name
    try:
        current_mission_request, current_mission_ip, ARGUMENTS = mission_map[mission_name]
    except Exception:
        current_mission_request = MissionNames.ERD
        print("invalid mission name, setting to ERD")
        return

    if current_mission_request == current_mission:
        return
    current_mission = current_mission_request

    for pipeline_number, pipeline in enumerate(__pipelines):
        # only skip if 0 or 1 because it's the same either way
        if pipeline_number == 0 or pipeline_number == 1:
            continue
        pipeline.update_video_output()


def camera_callback(channel, msg):
    global __pipelines

    port_devices = Cameras.decode(msg).port

    for port_number, requested_port_device in enumerate(port_devices):
        current_device_number = __pipelines[port_number].get_device_number()

        if current_device_number == requested_port_device:
            continue

        # check if we need to close current video source or not
        if pipeline_device_is_unique(port_number, current_device_number):
            close_video_source(current_device_number)

        create_video_source(requested_port_device)
        start_pipeline(requested_port_device, port_number)


def main():
    global __pipelines, __lcm

    __pipelines = [Pipeline(0), Pipeline(1), Pipeline(2), Pipeline(3)]

    __lcm = lcm.LCM()
    __lcm.subscribe("/cameras_cmd", camera_callback)
    __lcm.subscribe("/cameras_mission", mission_callback)

    while True:
        while __lcm.handle_timeout(0):
            pass
        for port_number, pipeline in enumerate(__pipelines):
            if pipeline.is_currently_streaming():
                pipeline.capture_and_render_image()


if __name__ == "__main__":
    main()
