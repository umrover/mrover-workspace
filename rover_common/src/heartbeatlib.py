import os
import asyncio
from rover_msgs import Heartbeat
from . import aiolcm


def gen_new_id():
    """
    Generates a random ACK ID for the heartbeat protocol.
    """
    return int.from_bytes(os.urandom(3), byteorder='big')


class Heartbeater:
    def __init__(self, publish, subscribe, callback, index):
        self.lcm_ = aiolcm.AsyncLCM()
        self.connected = False
        self.connection_state_changed = callback
        self.where = publish
        self.index = index

        self.lcm_.subscribe(subscribe, self.heartbeat_handler)

    def send_new(self):
        hb_message = Heartbeat()
        hb_message.new_ack_id = gen_new_id()
        self.lcm_.publish(self.where, hb_message.encode())

    async def loop(self):
        timeout = 2.0
        interval = 0.1
        while True:
            try:
                await self.lcm_.handle(timeout=timeout)
            except asyncio.TimeoutError:
                if self.connected:
                    self.connection_state_changed(False, self.index)
                self.connected = False

            if not self.connected:
                self.send_new()

            await asyncio.sleep(interval)

    def heartbeat_handler(self, channel, data):
        if not self.connected:
            self.connection_state_changed(True, self.index)
        self.connected = True
        in_msg = Heartbeat.decode(data)
        ret_msg = Heartbeat()
        ret_msg.recv_ack_id = in_msg.new_ack_id
        ret_msg.new_ack_id = gen_new_id()
        self.lcm_.publish(self.where, ret_msg.encode())


class OnboardHeartbeater(Heartbeater):
    def __init__(self, callback, index):
        super().__init__("/heartbeat/rover{}".format(index),
                         "/heartbeat/bs{}".format(index), callback, index)


class BaseStationHeartbeater(Heartbeater):
    def __init__(self, callback, index):
        super().__init__("/heartbeat/bs{}".format(index),
                         "/heartbeat/rover{}".format(index), callback, index)
