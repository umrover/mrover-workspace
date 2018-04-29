import rover_msgs
import sys
import functools
import pprint
import datetime
from lcm_tools_common import lcmutil
from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines


def usage():
    print('usage: {} TYPE_NAME CHANNEL'.format(sys.argv[0]))


def recv_message(type, channel, data):
    msg = lcmutil.decode(type, data)
    now = datetime.datetime.now()
    print('----- {}'.format(now.strftime('%Y-%m-%d %H:%M:%S.%f')))
    pprint.pprint(lcmutil.lcm_to_dict(msg))
    print()


def main():
    if len(sys.argv) < 3:
        usage()
        sys.exit(1)

    type_name = sys.argv[1]
    channel = sys.argv[2]

    if not hasattr(rover_msgs, type_name):
        print('error: no such type {}'.format(type_name))
        sys.exit(1)

    lcm_ = aiolcm.AsyncLCM()
    lcm_.subscribe(channel, functools.partial(recv_message, type_name))
    run_coroutines(lcm_.loop())
