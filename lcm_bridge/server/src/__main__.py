import asyncio
import websockets
import json

from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from . import lcmutil


class Bridge:
    def __init__(self):
        """
        Creates a Bridge.

        A Bridge consists of a Heartbeater and an AsyncLCM instance.
        """
        self.num_hbs = 7
        self.hbs = []
        for x in range(0, self.num_hbs):
            self.hbs.append(heartbeatlib.BaseStationHeartbeater(
                            self.connection_state_changed, x))
        self.connections = [False]*self.num_hbs
        self.lcm_ = aiolcm.AsyncLCM()
        self.subscriptions = {}

    def connection_state_changed(self, c, index):
        """
        Called when we either gain or lose connection.
        """
        self.connections[index] = c

    def publish(self, topic, message):
        """
        Publishes a message received from the WebSocket to a topic.
        """
        self.lcm_.publish(topic, lcmutil.dict_to_lcm(message))

    def add_subscription(self, topic, lcm_type, websocket):
        """
        Creates a subscription to a topic, which echoes over the
        WebSocket.
        """
        loop = asyncio.get_event_loop()

        def callback(topic, data):
            msg = lcmutil.decode(lcm_type, data)
            loop.create_task(websocket.send(json.dumps({
                'type': 'lcm_message',
                'topic': topic,
                'message': lcmutil.lcm_to_dict(msg)
            })))
        self.subscriptions[topic] = self.lcm_.subscribe(topic, callback)

    def remove_subscription(self, topic):
        """
        Removes a previously-created subscription.
        """
        self.lcm_.unsubscribe(self.subscriptions[topic])
        del self.subscriptions[topic]

    async def conn_state_pusher(self, websocket, path):
        """
        Coroutine that pushes changes in the connection state over the
        websocket.
        """
        while True:
            await websocket.send(json.dumps({
                'type': 'connection_state',
                'state': self.connections,
            }))
            await asyncio.sleep(2)

    async def lcm_bridge(self, websocket, path):
        """
        Handles LCM commands from the client.
        """
        while True:
            command = json.loads(await websocket.recv())
            if command['type'] == 'lcm_publish':
                self.publish(command['topic'], command['message'])
            elif command['type'] == 'lcm_subscribe':
                self.add_subscription(
                        command['topic'], command['lcm_type'], websocket)
            elif command['type'] == 'lcm_unsubscribe':
                self.remove_subscription(command['topic'])
            else:
                print('Invalid message type: {}'.format(command['type']))

    async def chatter(self, websocket, path):
        """
        Multiplexes the various WebSocket coroutines.
        """
        await asyncio.gather(
            self.conn_state_pusher(websocket, path),
            self.lcm_bridge(websocket, path)
        )

    async def main_loop(self):
        """
        Runs the main loop for a Bridge.
        """
        bridge_server = websockets.serve(self.chatter, '0.0.0.0', 8001)
        await asyncio.gather(
            bridge_server,
            self.lcm_.loop(),
            *(hb.loop() for hb in self.hbs)
        )


def main():
    bridge = Bridge()
    run_coroutines(bridge.main_loop())
