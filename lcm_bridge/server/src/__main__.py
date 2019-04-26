import asyncio
import websockets
import json

from rover_msgs import Joystick
from rover_common import heartbeatlib, aiolcm
from rover_common.aiohelper import run_coroutines
from lcm_tools_common import lcmutil


class Connection:

    def __init__(self, lcm_, websocket):
        self.websocket = websocket
        self.lcm_ = lcm_
        self.subscriptions = {}

    def subscribe(self, topic, lcm_type):
        loop = asyncio.get_event_loop()

        def callback(topic, data):
            msg = lcmutil.decode(lcm_type, data)
            loop.create_task(self.websocket.send(json.dumps({
                'type': 'lcm_message',
                'topic': topic,
                'message': lcmutil.lcm_to_dict(msg)
            })))
        self.subscriptions[topic] = self.lcm_.subscribe(topic, callback)

    def close(self):
        for subscription in self.subscriptions.values():
            self.lcm_.unsubscribe(subscription)


class Bridge:

    def __init__(self):
        """
        Creates a Bridge.

        A Bridge consists of a Heartbeater and an AsyncLCM instance.
        """
        self.num_hbs = 9
        self.hbs = []
        for x in range(0, self.num_hbs):
            self.hbs.append(heartbeatlib.BaseStationHeartbeater(
                            self.connection_state_changed, x))
        self.connections = [False]*self.num_hbs
        self.lcm_ = aiolcm.AsyncLCM()
        self.socket_connections = []
        self.home_page_connection = None

    def connection_state_changed(self, c, index):
        """
        Called when we either gain or lose connection.
        """
        self.connections[index] = c

    def publish(self, topic, message):
        """
        Publishes a message received from the WebSocket to a topic.
        """
        self.lcm_.publish(topic, lcmutil.dict_to_lcm(message).encode())

    async def _send_subscription(self, topic, msg, websocket):
        try:
            await websocket.send(json.dumps({
                'type': 'lcm_message',
                'topic': topic,
                'message': lcmutil.lcm_to_dict(msg)
            }))
        except websockets.exceptions.ConnectionClosed as e:
            print('Websocket connection lost: {}'.format(str(e)))
            return

    def add_subscription(self, topic, lcm_type, websocket):
        """
        Creates a subscription to a topic, which echoes over the
        WebSocket.
        """
        loop = asyncio.get_event_loop()

        def callback(topic, data):
            msg = lcmutil.decode(lcm_type, data)
            loop.create_task(self._send_subscription(topic, msg, websocket))
        self.subscriptions[topic] = self.lcm_.subscribe(topic, callback)

    def remove_subscription(self, topic):
        """
        Removes a previously-created subscription.
        """
        self.lcm_.unsubscribe(self.subscriptions[topic])
        del self.subscriptions[topic]

    def clean_subscriptions(self):
        """
        Erases all subscriptions.
        """
        for sub in self.subscriptions.values():
            self.lcm_.unsubscribe(sub)
        self.subscriptions.clear()

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

    async def lcm_bridge(self, connection, path):
        """
        Handles LCM commands from the client.
        """
        while True:
            try:
                command = json.loads(await connection.websocket.recv())
                if command['type'] == 'lcm_publish':
                    self.publish(command['topic'], command['message'])
                elif command['type'] == 'lcm_subscribe':
                    connection.subscribe(
                        command['topic'], command['lcm_type'])
                elif command['type'] == 'home_page_set':
                    self.home_page_connection = connection
                else:
                    print('Invalid message type: {}'.format(command['type']))
            except websockets.exceptions.ConnectionClosed as e:
                raise e
            except Exception as e:
                await connection.websocket.send(json.dumps({
                    'type': 'error_message',
                    'message': "Error when sending command: {}\n{}"
                    .format(str(command), str(e))
                }))
                print('exception: {}'.format(str(e)))

    async def chatter(self, websocket, path):
        """
        Multiplexes the various WebSocket coroutines.
        """
        try:
            connection = Connection(self.lcm_, websocket)
            self.socket_connections.append(connection)

            await asyncio.gather(
                self.conn_state_pusher(websocket, path),
                self.lcm_bridge(connection, path)
            )
        except websockets.exceptions.ConnectionClosed as e:
            connection = None
            for i in range(len(self.socket_connections)):
                if self.socket_connections[i].websocket == websocket:
                    connection = self.socket_connections[i]
                    del self.socket_connections[i]
                    break

            connection.close()
            if self.home_page_connection == connection:
                self.home_page_connection = None

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

    async def send_kills(self):
        while True:
            if self.home_page_connection is None:
                js = Joystick()
                js.kill = True
                js.forward_back = 0
                js.left_right = 0
                self.lcm_.publish("/drive_control", js.encode())

            await asyncio.sleep(0.5)


def main():
    bridge = Bridge()
    run_coroutines(bridge.main_loop(), bridge.send_kills())
