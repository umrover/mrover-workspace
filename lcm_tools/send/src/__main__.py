import sys
import json
from lcm_tools_common import lcmutil
from rover_common import aiolcm


def usage():
    print('usage: {} CHANNEL MSG\n'.format(sys.argv[0]))


def main():
    if len(sys.argv) < 3:
        usage()
        sys.exit(1)

    channel = sys.argv[1]
    msg = json.loads(" ".join(sys.argv[2:]).replace("\'", "\""))

    lcm_ = aiolcm.AsyncLCM()
    lcm_.publish(channel, lcmutil.dict_to_lcm(msg).encode())
