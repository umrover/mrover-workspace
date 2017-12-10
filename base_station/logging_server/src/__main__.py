from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Sensors
from datetime import datetime
import os

print_order = ["temperature", "moisture", "soil_conductivity", "pH", "uv"]
tabs = [2, 2, 1, 3, 3]
lcm_ = aiolcm.AsyncLCM()
out = None


def sensor_callback(channel, msg):
    global out
    data = Sensors.decode(msg)
    out.write("{}:\n".format(str(datetime.now())))
    for i in range(5):
        item = print_order[i]
        out.write("\t{}:{}{}\n".format(item, "\t"*tabs[i],
                                       getattr(data, item)))
    out.flush()
    os.fsync(out.fileno())


def main():
    global out
    parent_dir = os.path.join(os.environ['HOME'], 'mrover-data')
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
    path = os.path.join(parent_dir, "SensorData.txt")
    out = open(path, "a+")
    out.write("\nOpening file at {}\n".format(str(datetime.now())))
    out.flush()
    os.fsync(out.fileno())

    lcm_.subscribe("/sensors", sensor_callback)
    run_coroutines(lcm_.loop())
