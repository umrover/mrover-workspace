import jetson.utils
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument("input_URI", type=str, help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs="?", help="URI of the output stream")
opt = parser.parse_known_args()[0]

input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)

while output.IsStreaming():
    image = input.Capture(format='rgb8')
    output.Render(image)
    output.SetStatus("Video Viewer | {:d}x{:d} | {:.1} FPS".format(image.width, image.height, output.GetFrameRate()))
