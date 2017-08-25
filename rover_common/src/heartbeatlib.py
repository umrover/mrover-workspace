import os
from rover_msgs import Heartbeat


def gen_new_id():
    """
    Generates a random ACK ID for the heartbeat protocol.
    """
    return int.from_bytes(os.urandom(3), byteorder='big')


class Heartbeater:
    def __init__(self, lcm_, publish, subscribe):
        self.lcm_ = lcm_
        self.is_connected = False
        self.where = publish

        self.lcm_.subscribe(subscribe, self.heartbeat_handler)

    def send_new(self):
        hb_message = Heartbeat()
        hb_message.new_ack_id = gen_new_id()
        self.lcm_.publish(self.where, hb_message.encode())

    def timeout(self):
        self.is_connected = False

    def heartbeat_handler(self, channel, data):
        self.is_connected = True
        in_msg = Heartbeat.decode(data)
        ret_msg = Heartbeat()
        ret_msg.recv_ack_id = in_msg.new_ack_id
        ret_msg.new_ack_id = gen_new_id()
        self.lcm_.publish(self.where, ret_msg.encode())


class OnboardHeartbeater(Heartbeater):
    def __init__(self, lcm_):
        super().__init__(lcm_, "/heartbeat/rover", "/heartbeat/bs")


class BaseStationHeartbeater(Heartbeater):
    def __init__(self, lcm_):
        super().__init__(lcm_, "/heartbeat/bs", "/heartbeat/rover")
