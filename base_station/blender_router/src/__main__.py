import socket
import threading
import json

from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import ArmPosition, IkArmControl, OpenLoopRAMotor

lcm_ = aiolcm.AsyncLCM()

VAGRANT = True

ik_mode = True

NUM_PREV_MESSAGES = 10
prev_messages = [None]*NUM_PREV_MESSAGES


def listen_blender():
    global ik_mode
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', 8019))
    sock.listen(5)
    conn, addr = sock.accept()

    while True:
        try:
            # conn, addr = sock.accept()
            rawData = conn.recv(1024).decode()
            try:
                data = json.loads(rawData)
            except:
                continue

            diff_count = 0
            if prev_messages[0] is None:
                diff_count = NUM_PREV_MESSAGES
            else:
                for pd in prev_messages:
                    for i in range(5):
                        j = chr(ord('A')+i)
                        if pd is None or prev_messages[0][j] != pd[j]:
                            diff_count += 1
                            break

            prev_messages.pop(0)
            prev_messages.append(data)

            if diff_count <= 1:
                continue

            # Send data to onboard rover
            msg = ArmPosition()
            msg.joint_a = -data['A']
            msg.joint_b = -data['B']
            msg.joint_c = -data['C']
            msg.joint_d = -data['D']
            msg.joint_e = -data['E']

            lcm_.publish('/ik_ra_control', msg.encode())

        except socket.error as exc:
            print(exc)
            pass


def ik_callback(channel, msg):
    global ik_mode
    # print("recv xbox msg")
    ik_mode = True
    deltas = IkArmControl.decode(msg)
    data = {
        "type": "IK",
        "deltaX": deltas.deltaX,
        "deltaY": deltas.deltaY,
        "deltaZ": deltas.deltaZ,
        "deltaTilt": deltas.deltaTilt,
        "deltaJointE": deltas.deltaJointE
    }
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if VAGRANT:
            sock.connect(("10.0.2.2", 8018))
        else:
            sock.connect(("127.0.0.1", 8018))
        sock.send(json.dumps(data).encode())
    except socket.error as exc:
        print(exc)
        pass


def fk_callback(channel, msg):
    if ik_mode:
        return

    angles = ArmPosition.decode(msg)
    data = {
        "type": "FK",
        "joint_a": angles.joint_a,
        "joint_b": angles.joint_b,
        "joint_c": angles.joint_c,
        "joint_d": angles.joint_d,
        "joint_e": angles.joint_e,
    }
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if VAGRANT:
            sock.connect(("10.0.2.2", 8018))
        else:
            sock.connect(("127.0.0.1", 8018))
        sock.send(json.dumps(data).encode())
    except socket.error as exc:
        # print(exc)
        pass


def open_loop_callback(channel, msg):
    global ik_mode
    joint_id = OpenLoopRAMotor.decode(msg).joint_id
    if joint_id < 5:
        ik_mode = False


def main():
    lcm_.subscribe("/ik_arm_control", ik_callback)
    lcm_.subscribe("/arm_position", fk_callback)
    lcm_.subscribe('/arm_motors', open_loop_callback)
    thread = threading.Thread(target=listen_blender)
    thread.start()
    run_coroutines(lcm_.loop())
