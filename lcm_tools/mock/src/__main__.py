from random import normalvariate
from lcm_tools_common import lcmutil
from rover_common import aiolcm
import time


def main():
    lcm_ = aiolcm.AsyncLCM()

    pdb_channel = '/pdb_data'
    pdb_msg = {
        'type': 'PDBData',
        'temp': [0, 0, 0],
        'current': [0, 0, 0],
        'voltage': [0, 0, 0]
    }

    fuse_channel = '/fuse_data'
    fuse_msg = {
        'type': 'FuseData',
        'current': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        'voltage': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    while True:

        pdb_msg['temp'][0] = normalvariate(45.05, 0.55)
        pdb_msg['temp'][1] = normalvariate(49.8, 0.7)
        pdb_msg['temp'][2] = normalvariate(65.2, 0.5)

        pdb_msg['current'][0] = normalvariate(1.5, 0.01)
        pdb_msg['current'][1] = normalvariate(3.4, 0.03)
        pdb_msg['current'][2] = normalvariate(5.2, 0.03)

        pdb_msg['voltage'][0] = normalvariate(3.2815, 0.001)
        pdb_msg['voltage'][1] = normalvariate(4.899, 0.001)
        pdb_msg['voltage'][2] = normalvariate(11.805, 0.0015)

        lcm_.publish(pdb_channel, lcmutil.dict_to_lcm(pdb_msg).encode())

        fuse_msg['current'][0] = normalvariate(1.5, 0.01)
        fuse_msg['current'][1] = normalvariate(1.4, 0.03)
        fuse_msg['current'][2] = normalvariate(1.2, 0.03)
        fuse_msg['current'][3] = normalvariate(1.5, 0.01)
        fuse_msg['current'][4] = normalvariate(1.4, 0.03)
        fuse_msg['current'][5] = normalvariate(1.2, 0.03)
        fuse_msg['current'][6] = normalvariate(1.5, 0.01)
        fuse_msg['current'][7] = normalvariate(1.4, 0.03)
        fuse_msg['current'][8] = normalvariate(1.2, 0.03)
        fuse_msg['current'][9] = normalvariate(1.5, 0.01)
        fuse_msg['current'][10] = normalvariate(1.4, 0.03)
        fuse_msg['current'][11] = normalvariate(1.2, 0.03)

        fuse_msg['voltage'][0] = normalvariate(11.2, 0.1)
        fuse_msg['voltage'][1] = normalvariate(15.1, 0.1)
        fuse_msg['voltage'][2] = normalvariate(12.6, 0.2)
        fuse_msg['voltage'][3] = normalvariate(3.2, 0.1)
        fuse_msg['voltage'][4] = normalvariate(25.1, 0.1)
        fuse_msg['voltage'][5] = normalvariate(13.6, 0.2)
        fuse_msg['voltage'][6] = normalvariate(33.2, 0.1)
        fuse_msg['voltage'][7] = normalvariate(35.1, 0.1)
        fuse_msg['voltage'][8] = normalvariate(22.6, 0.2)
        fuse_msg['voltage'][9] = normalvariate(1.2, 0.1)
        fuse_msg['voltage'][10] = normalvariate(5.1, 0.1)
        fuse_msg['voltage'][11] = normalvariate(12.6, 0.2)

        lcm_.publish(fuse_channel, lcmutil.dict_to_lcm(fuse_msg).encode())

        time.sleep(0.1)
