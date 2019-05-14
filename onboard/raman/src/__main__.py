import queue
import os
import time
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines

from .config import Config
from .CCDserial import rxtxonce
from .CCDfiles import savefile


config = Config()
lcm_ = aiolcm.AsyncLCM()


def collect_callback(channel, msg):
    SerQueue = queue.Queue()
    rxtxonce(SerQueue, config)
    savefile(config)
    os.system('scp -l 2000 RAW.dat mrover@10.0.0.1:raman_{}.dat'
              .format(round(time.time() * 1000)))


def main():
    lcm_.subscribe("/raman_collect", collect_callback)

    run_coroutines(lcm_.loop())
