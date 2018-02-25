import asyncio
import socket
import struct


# CAN flags
CAN_EFF_FLAG = 0x80000000

# BCM opcodes
CAN_BCM_TX_SETUP = 1
CAN_BCM_TX_DELETE = 2
CAN_BCM_TX_SEND = 4
CAN_BCM_RX_SETUP = 5
CAN_BCM_RX_DELETE = 6
CAN_BCM_RX_CHANGED = 12


# struct bcm_msg_head
BCM_MSG_HEAD_FMT = "@3I4l2I0q"


# BCM flags
SETTIMER = 0x0001
STARTTIMER = 0x0002
TX_COUNTEVT = 0x0004
TX_ANNOUNCE = 0x0008
TX_CP_CAN_ID = 0x0010
RX_FILTER_ID = 0x0020
RX_CHECK_DLC = 0x0040
RX_NO_AUTOTIMER = 0x0080
RX_ANNOUNCE_RESUME = 0x0100
TX_RESET_MULTI_IDX = 0x0200
RX_RTR_FRAME = 0x0400
CAN_FD_FRAME = 0x0800


def arb_id_to_can_id(arb_id, extended):
    if extended:
        return arb_id | CAN_EFF_FLAG
    return arb_id


def can_id_to_arb_id(can_id):
    if bool(can_id & CAN_EFF_FLAG):
        return can_id & 0x1FFFFFFF
    else:
        return can_id & 0x000007FF


class Message:
    FMT = "=IB3x8s"
    MAX_DLEN = 8

    def __init__(self, arb_id, data, extended_id=False):
        if len(data) > Message.MAX_DLEN:
            raise ValueError("data too long for CAN message")
        self.arb_id = arb_id
        self.data = bytes(data)
        self.extended_id = extended_id

    def to_raw(self):
        if not isinstance(self.data, bytes):
            self.data = bytes(self.data)
        can_dlc = len(self.data)
        data = self.data.ljust(8, b'\x00')
        can_id = arb_id_to_can_id(self.arb_id, self.extended_id)
        return struct.pack(Message.FMT, can_id, can_dlc, data)

    @classmethod
    def from_raw(cls, frame):
        can_id, can_dlc, data = struct.unpack(Message.FMT, frame)
        return Message(can_id_to_arb_id(can_id), data[:can_dlc])

    def __str__(self):
        return '{} [{}] {}'.format(
                hex(self.arb_id), len(self.data),
                ' '.join(hex(b) for b in self.data))


def bcm_header(opcode, flags, count, ival1_sec, ival1_usec,
               ival2_sec, ival2_usec, can_id, nframes):
    return struct.pack(BCM_MSG_HEAD_FMT, opcode, flags, count, ival1_sec,
                       ival1_usec, ival2_sec, ival2_usec, can_id, nframes)


def bcm_tx_setup_header(can_id, count, initial_period, subsequent_period,
                        msg_flags):
    opcode = CAN_BCM_TX_SETUP
    flags = msg_flags | SETTIMER | STARTTIMER

    if initial_period > 0:
        flags |= TX_COUNTEVT

    def split_time(val):
        sec = int(val)
        usec = int(1e6 * (val - sec))
        return sec, usec

    ival1_sec, ival1_usec = split_time(initial_period)
    ival2_sec, ival2_usec = split_time(subsequent_period)
    nframes = 1

    return bcm_header(opcode, flags, count, ival1_sec, ival1_usec,
                      ival2_sec, ival2_usec, can_id, nframes)


def bcm_tx_delete_header(can_id, flags):
    opcode = CAN_BCM_TX_DELETE
    return bcm_header(opcode, flags, 0, 0, 0, 0, 0, can_id, 1)


class BCMPeriodicTask:
    def __init__(self, sock, frame, period):
        self.sock = sock
        self.frame = frame
        self.period = period

    async def _tx_setup(self):
        loop = asyncio.get_event_loop()
        header = bcm_tx_setup_header(self.frame.arb_id, 0, 0.0, self.period, 0)
        frame = self.frame.to_raw()
        await loop.sock_sendall(self.sock, header + frame)

    async def start(self):
        await self._tx_setup()

    async def stop(self):
        loop = asyncio.get_event_loop()
        stopframe = bcm_tx_delete_header(self.frame.arb_id, 0)
        await loop.sock_sendall(self.sock, stopframe)

    async def modify(self, frame):
        self.frame = frame
        await self._tx_setup()


class AsyncBCM:
    def __init__(self, iface):
        self.sock = socket.socket(
                socket.PF_CAN, socket.SOCK_DGRAM, socket.CAN_BCM)
        self.sock.setblocking(False)
        self.sock.connect((iface,))
        self._subscriptions = {}

    async def subscribe(self, arb_id, cb, extended=False):
        loop = asyncio.get_event_loop()
        opcode = CAN_BCM_RX_SETUP
        can_id = arb_id_to_can_id(arb_id, extended)
        flags = RX_FILTER_ID
        msg = bcm_header(opcode, flags, 0, 0, 0, 0, 0, can_id, 1)
        await loop.sock_sendall(self.sock, msg)
        self._subscriptions[arb_id] = cb
        return arb_id

    async def unsubscribe(self, arb_id, extended=False):
        loop = asyncio.get_event_loop()
        opcode = CAN_BCM_RX_DELETE
        can_id = arb_id_to_can_id(arb_id, extended)
        msg = bcm_header(opcode, 0, 0, 0, 0, 0, 0, can_id, 1)
        await loop.sock_sendall(self.sock, msg)
        del self._subscriptions[arb_id]

    async def send(self, frame):
        loop = asyncio.get_event_loop()
        opcode = CAN_BCM_TX_SEND
        can_id = arb_id_to_can_id(frame.arb_id, frame.extended_id)
        msg = bcm_header(opcode, 0, 0, 0, 0, 0, 0, can_id, 1)
        await loop.sock_sendall(self.sock, msg + frame.to_raw())

    async def send_periodic(self, frame, period):
        task = BCMPeriodicTask(self.sock, frame, period)
        await task.start()
        return task

    async def loop(self):
        loop = asyncio.get_event_loop()
        while True:
            in_msg = await loop.sock_recv(self.sock, 72)
            header = struct.unpack_from(BCM_MSG_HEAD_FMT, in_msg)
            opcode = header[0]
            can_id = header[7]
            if opcode == CAN_BCM_RX_CHANGED:
                arb_id = can_id_to_arb_id(can_id)
                assert arb_id in self._subscriptions
                msg = Message.from_raw(
                        in_msg[struct.calcsize(BCM_MSG_HEAD_FMT):])
                loop.call_soon(self._subscriptions[arb_id], msg)
