import sys

from rover_common import heartbeatlib
from rover_common.aiohelper import run_coroutines


def connection_state_changed(c, _):
    if c:
        print("Connection established.")
    else:
        print("Disconnected.")


def main():
    hb = heartbeatlib.OnboardHeartbeater(connection_state_changed, sys.argv[1])
    run_coroutines(hb.loop())
