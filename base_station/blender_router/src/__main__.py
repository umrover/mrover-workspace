import socket
import asyncio
import json

from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import Encoder

lcm_ = aiolcm.AsyncLCM()


async def listen_blender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', 8019))
    sock.listen(1)
    sock.settimeout(1)

    conn = None
    while conn is None:
        try:
            conn, addr = sock.accept()
        except socket.timeout:
            await asyncio.sleep(0.1)

    while True:
        try:
            rawData = str(conn.recv(1024)).replace('\'', '"')
            rawData = rawData[2:len(rawData)-1]
            data = json.loads(rawData)

            # Send data to onboard rover
            msg = Encoder()
            msg.joint_a = int(data['A'])
            msg.joint_b = int(data['B'])
            msg.joint_c = int(data['C'])
            msg.joint_d = int(data['D'])
            msg.joint_e = int(data['E'])
            msg.joint_f = 0

            print(msg)

            lcm_.publish('/ik_ra_control', msg.encode())

        except socket.timeout:
            await asyncio.sleep(0.1)


def main():
    run_coroutines(lcm_.loop(), listen_blender())
