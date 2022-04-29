from rover_msgs import Cameras
import lcm

import sys
sys.path.insert(0, "/usr/lib/python3.6/dist-packages")  # 3.6 vs 3.8
import jetson.utils  # noqa

__lcm: lcm.LCM
__pipelines = [None] * 2

ARGUMENTS_LOW = ['--headless', '--bitrate=300000', '--width=256', '--height=144']
ARGUMENTS_MEDIUM = ['--headless', '--bitrate=300000', '--width=1280', '--height=720']
remote_ip = ["10.0.0.1:5000", "10.0.0.1:5001", "10.0.0.2:5000", "10.0.0.2:5001"]


def main():
    video_source_0 = jetson.utils.videoSource(f"/dev/video0", argv=ARGUMENTS_MEDIUM)
    video_source_1 = jetson.utils.videoSource(f"/dev/video1", argv=ARGUMENTS_MEDIUM)
    video_source_2 = jetson.utils.videoSource(f"/dev/video2", argv=ARGUMENTS_MEDIUM)
    video_source_3 = jetson.utils.videoSource(f"/dev/video3", argv=ARGUMENTS_MEDIUM)
    # video_source_4 = jetson.utils.videoSource(f"/dev/video4", argv=ARGUMENTS_MEDIUM)
    # video_source_5 = jetson.utils.videoSource(f"/dev/video5", argv=ARGUMENTS_MEDIUM)
    # video_source_6 = jetson.utils.videoSource(f"/dev/video6", argv=ARGUMENTS_MEDIUM)
    # video_source_7 = jetson.utils.videoSource(f"/dev/video7", argv=ARGUMENTS_MEDIUM)
    video_output_0 = jetson.utils.videoOutput(f"rtp://10.0.0.1:5000", argv=ARGUMENTS_MEDIUM)
    video_output_1 = jetson.utils.videoOutput(f"rtp://10.0.0.1:5001", argv=ARGUMENTS_MEDIUM)
    video_output_2 = jetson.utils.videoOutput(f"rtp://10.0.0.1:5002", argv=ARGUMENTS_MEDIUM)
    video_output_3 = jetson.utils.videoOutput(f"rtp://10.0.0.1:5003", argv=ARGUMENTS_MEDIUM)

    while True:
        image_0 = video_source_0.Capture()
        video_output_0.Render(image_0)
        image_1 = video_source_1.Capture()
        video_output_1.Render(image_1)
        # image_0 = video_source_0.Capture()
        # video_output_0.Render(image_0)
        # video_output_1.Render(image_0)


if __name__ == "__main__":
    main()
