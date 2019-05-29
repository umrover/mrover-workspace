import queue
import os
import time
import lcm

from .config import Config
from .CCDserial import rxtxonce
from .CCDfiles import savefile


config = Config()


def collect_callback(channel, msg):
    SerQueue = queue.Queue()
    rxtxonce(SerQueue, config)
    savefile(config)
    os.system(('scp -l 2000 /tmp/INT.dat '
               'mrover@10.0.0.2:science-data/Raman/{}.dat')
              .format(round(time.time() * 1000)))


def main():
    lcm_ = lcm.LCM()
    lcm_.subscribe("/raman_collect", collect_callback)

    try:
        while True:
            lcm_.handle()
    except KeyboardInterrupt:
        pass
