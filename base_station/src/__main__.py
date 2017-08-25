import lcm
import select
import time
from rover_common import heartbeatlib

# TODO refactor this code
def main():
    lc = lcm.LCM()
    hb = heartbeatlib.BaseStationHeartbeater(lc)

    timeout = 2.0
    period = 0.1
    times_between_prints = 10
    i = 0
    try:
        while True:
            rfds, _wfds, _efds = select.select([lc.fileno()], [], [], timeout)
            if rfds:
                lc.handle()
            else:
                hb.timeout()
                print("Not connected.")

            if i == times_between_prints:
                if hb.is_connected:
                    print("Connected to rover.")
                i = 0

            time.sleep(period)
            i = i + 1
    except KeyboardInterrupt:
        pass
