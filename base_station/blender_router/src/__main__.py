import socket
import threading
import json

from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import ArmPosition, IkArmControl

lcm_ = aiolcm.AsyncLCM()


def listen_blender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', 8019))
    sock.listen(5)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    conn, addr = sock.accept()

    while True:
        try:
            # conn, addr = sock.accept()
            rawData = conn.recv(1024).decode()
            data = json.loads(rawData)

            # Send data to onboard rover
            msg = ArmPosition()
            msg.joint_a = data['A']
            msg.joint_b = data['B']
            msg.joint_c = data['C']
            msg.joint_d = data['D']
            msg.joint_e = data['E']

            lcm_.publish('/ik_ra_control', msg.encode())

        except socket.error as exc:
            # print(exc)
            pass


def ik_callback(channel, msg):
    # print("recv xbox msg")
    deltas = IkArmControl.decode(msg)
    data = {
        "deltaX": deltas.deltaX,
        "deltaY": deltas.deltaY,
        "deltaZ": deltas.deltaZ
    }
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("10.0.2.2", 8018))
        sock.send(json.dumps(data).encode())
    except socket.error as exc:
        # print(exc)
        pass


def main():
    lcm_.subscribe("/ik_arm_control", ik_callback)
    thread = threading.Thread(target=listen_blender)
    thread.start()
    run_coroutines(lcm_.loop())
