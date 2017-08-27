import asyncio
import lcm


class AsyncLCM:
    def __init__(self, *args, **kwargs):
        self.lcm_ = lcm.LCM(*args, **kwargs)

    def publish(self, topic, data):
        self.lcm_.publish(topic, data)

    def subscribe(self, topic, callback=None):
        return self.lcm_.subscribe(topic, callback)

    def unsubscribe(self, subscription):
        self.lcm_.unsubscribe(subscription)

    async def handle(self, *, timeout=None):
        loop = asyncio.get_event_loop()
        handled_queue = asyncio.Queue()

        def response():
            self.lcm_.handle()
            loop.create_task(handled_queue.put(None))

        loop.add_reader(self.lcm_.fileno(), response)

        try:
            if timeout is None:
                return await handled_queue.get()
            else:
                return await asyncio.wait_for(handled_queue.get(), timeout)
        finally:
            loop.remove_reader(self.lcm_.fileno())

    async def loop(self):
        while True:
            await self.handle()
