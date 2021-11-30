"""
https://github.com/dusty-nv/jetson-utils
TODO: figure out how to import this Python library (may need to be built from CMakeLists.txt)
TODO: add rover messages, not really sure how to do this
"""

import jetson.utils

VIDEO_DEVICE = "/dev/video0"
STREAM_IP = "10.0.0.1"
STREAM_PORT = 5000
STREAM_ADDRESS = f"rtp://{STREAM_IP}:{STREAM_PORT}"


def main():
    source = jetson.utils.videoSource(VIDEO_DEVICE)
    output = jetson.utils.videoOutput(STREAM_ADDRESS)
    while output.IsStreaming():
        image = source.Capture()
        output.Render(image)
        output.SetStatus(f"Video Viewer | {image.width:d}x{image.height:d} | {output.GetFrameRate():.1f} FPS")
