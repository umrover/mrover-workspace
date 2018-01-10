from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Sensors, SensorSwitch
import os
import csv


print_order = ["timestamp", "temperature", "moisture",
               "soil_conductivity", "pH", "uv"]
lcm_ = aiolcm.AsyncLCM()
outFile = None
outWriter = None
should_record = False


def switch_callback(channel, msg):
    global should_record
    should_record = SensorSwitch.decode(msg).should_record


def sensor_callback(channel, msg):
    global should_record
    if not should_record:
        return

    global outFile
    global outWriter

    data = Sensors.decode(msg)
    writeData = [getattr(data, print_order[i]) for i in range(5)]

    outWriter.writerow(writeData)

    outFile.flush()
    os.fsync(outFile.fileno())


def main():
    global outFile
    global outWriter

    parent_dir = os.path.join(os.environ['HOME'], 'mrover-data')
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
    path = os.path.join(parent_dir, "SensorData.csv")

    needsHeader = not os.path.exists(path)

    outFile = open(path, 'a+', newline='')
    outWriter = csv.writer(outFile, delimiter=',',
                           quotechar='|', quoting=csv.QUOTE_MINIMAL)
    if needsHeader:
        outWriter.writerow(print_order)

        outFile.flush()
        os.fsync(outFile.fileno())

    lcm_.subscribe("/sensor_switch", switch_callback)
    lcm_.subscribe("/sensors", sensor_callback)
    run_coroutines(lcm_.loop())
