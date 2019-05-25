from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import RGB, RGBFrame
import os
import csv
import time


header = ["timestamp", "r", "g", "b"]
lcm_ = aiolcm.AsyncLCM()


parent_dir = None
out_data = {}

last_rgb_vals = {
    "rgb_ammonia_1": {
        'r': 0,
        'g': 0,
        'b': 0
    },
    "rgb_ammonia_2": {
        'r': 0,
        'g': 0,
        'b': 0
    },
    "rgb_buret_1": {
        'r': 0,
        'g': 0,
        'b': 0
    },
    "rgb_buret_2": {
        'r': 0,
        'g': 0,
        'b': 0
    },
}


def rgb_callback(channel, msg):
    global last_rgb_vals
    rgb = RGB.decode(msg)
    last_rgb_vals[rgb.id] = {
        'r': rgb.r,
        'g': rgb.g,
        'b': rgb.b
    }


def rgb_frame_callback(channel, msg):
    global out_data

    rgb_frame = RGBFrame.decode(msg)
    if rgb_frame.id not in out_data:
        path = os.path.join(parent_dir, rgb_frame.id+'.csv')
        needsHeader = not os.path.exists(path)

        outFile = open(path, 'a+', newline='')
        outWriter = csv.writer(outFile, delimiter=',',
                               quotechar='|', quoting=csv.QUOTE_MINIMAL)
        if needsHeader:
            outWriter.writerow(header)
            outFile.flush()
            os.fsync(outFile.fileno())

        out_data[rgb_frame.id] = (outFile, outWriter)

    outFile, outWriter = out_data[rgb_frame.id]

    outWriter.writerow(
        [int(time.time()),
         last_rgb_vals[rgb_frame.id]['r'],
         last_rgb_vals[rgb_frame.id]['g'],
         last_rgb_vals[rgb_frame.id]['b']]
    )

    outFile.flush()
    os.fsync(outFile.fileno())


def make_science_directories():
    global parent_dir

    parent_dir = os.path.join(os.environ['HOME'], 'science-data/Tests')
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
    microscope_dir = os.path.join(os.environ['HOME'],
                                  'science-data/Microscope')
    if not os.path.exists(microscope_dir):
        os.makedirs(microscope_dir)
    microcam_dir = os.path.join(os.environ['HOME'], 'science-data/MicroCam')
    if not os.path.exists(microcam_dir):
        os.makedirs(microcam_dir)
    pi_dir = os.path.join(os.environ['HOME'], 'science-data/PiPictures')
    if not os.path.exists(pi_dir):
        os.makedirs(pi_dir)
    raman_dir = os.path.join(os.environ['HOME'], 'science-data/Raman')
    if not os.path.exists(raman_dir):
        os.makedirs(raman_dir)


def main():
    global out_data

    make_science_directories()

    lcm_.subscribe("/rgb_frame", rgb_frame_callback)
    lcm_.subscribe("/rgb", rgb_callback)
    run_coroutines(lcm_.loop())
