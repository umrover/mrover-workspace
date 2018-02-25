import struct
import asyncio
from rover_common.aiohelper import wait_for
from rover_common import talon_srx, aiocan


class MessageBuffer:
    def __init__(self, arb_id, maxsize=0):
        self.arb_id = arb_id
        self.buffer = asyncio.Queue(maxsize)

    def __call__(self, msg):
        assert msg.arb_id == self.arb_id
        self.buffer.put_nowait(msg)

    async def get(self, timeout=0.5):
        return await asyncio.wait_for(self.buffer.get(), timeout=timeout)


class SingleMessageBuffer:
    def __init__(self, arb_id):
        self.arb_id = arb_id
        self.evt = asyncio.Event()
        self.evt.clear()
        self.msg = None

    def __call__(self, msg):
        assert msg.arb_id == self.arb_id
        self.msg = msg
        self.evt.set()

    async def get(self):
        await self.evt.wait()
        return self.msg


class LowLevel(object):
    def __init__(self, iface, device_id):
        self.device_id = device_id
        self.bus = aiocan.AsyncBCM(iface)
        self.param_response_buffer = MessageBuffer(
                talon_srx.PARAM_RESPONSE | device_id)
        self.status_3_buffer = SingleMessageBuffer(
                talon_srx.STATUS_3 | device_id)
        wait_for(self._setup())

    async def _setup(self):
        await self.bus.subscribe(
                talon_srx.PARAM_RESPONSE | self.device_id,
                self.param_response_buffer, extended=True)
        await self.bus.subscribe(
                talon_srx.STATUS_3 | self.device_id,
                self.status_3_buffer, extended=True)
        # for externalenable being false, control_1 is needed
        self.msg_control_1 = aiocan.Message(
                    arb_id=talon_srx.CONTROL_1 | self.device_id,
                    extended_id=True,
                    data=[0 for i in range(0, 2)])
        self.control_1_task = await self.bus.send_periodic(
                self.msg_control_1, 0.05)
        self.msg_control_5 = aiocan.Message(
                    arb_id=talon_srx.CONTROL_5 | self.device_id,
                    extended_id=True,
                    data=[0 for i in range(0, 8)])
        self.control_5_task = await self.bus.send_periodic(
                self.msg_control_5, 0.01)
        await self.set_override_limit_switches(
                talon_srx.kLimitSwitchOverride.EnableFwd_EnableRev.value)

    async def set_param(self, param_id, param_val):
        frame = 0
        if param_id in [
                talon_srx.Param.ProfileParamSlot0_P,
                talon_srx.Param.ProfileParamSlot0_I,
                talon_srx.Param.ProfileParamSlot0_D,
                talon_srx.Param.ProfileParamSlot1_P,
                talon_srx.Param.ProfileParamSlot1_I,
                talon_srx.Param.ProfileParamSlot1_D]:
            frame = talon_srx.float_to_fxp_10_22(param_val) & 0xFFFFFFFF
        elif param_id in [
                talon_srx.Param.ProfileParamSlot0_F,
                talon_srx.Param.ProfileParamSlot1_F]:
            param_val = min(max(param_val, -255), 255)
        else:
            frame = int(param_val)

        frame <<= 8
        frame |= param_id
        arbId = talon_srx.PARAM_SET | self.device_id
        msg = aiocan.Message(arb_id=arbId,
                             extended_id=True,
                             data=struct.pack('<Q', frame)[:5])
        await self.bus.send(msg)
        while True:
            msg = await self.param_response_buffer.get(timeout=None)
            if msg.data[0] == param_id:
                return

    async def read_enc_value(self):
        msg = await self.status_3_buffer.get()
        if msg is None:
            return None
        _cache = struct.unpack('<Q', msg.data)[0]
        H = (_cache >> 0) & 0xff
        M = (_cache >> 8) & 0xff
        L = (_cache >> 16) & 0xff
        raw = 0
        raw |= H
        raw <<= 8
        raw |= M
        raw <<= 8
        raw |= L
        raw = talon_srx.sign_extend(raw, 24)
        return raw

    async def set_override_brake_type(self, param):
        param = int(param) & 0x3
        frame = struct.unpack('<Q', self.msg_control_5.data)[0]
        frame &= ~(0x3 << 50)
        frame |= param << 50
        self.msg_control_5.data = struct.pack('<Q', frame)[:8]
        await self.control_5_task.modify(self.msg_control_5)

    async def set_demand(self, param, controlmode):
        param = int(param)
        H = (param >> 16) & 0xff
        M = (param >> 8) & 0xff
        L = (param) & 0xff

        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        _cache &= ~(0xFF << 16)
        _cache &= ~(0xFF << 24)
        _cache &= ~(0xFF << 32)
        _cache |= H << 16
        _cache |= M << 24
        _cache |= L << 32

        controlmode = controlmode & 0xf
        _cache &= ~((0xf) << 52)
        _cache |= controlmode << 52
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        await self.control_5_task.modify(self.msg_control_5)

    async def set_profile_slot_select(self, param):
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        if param == 0:
            _cache &= ~(1 << 40)
        else:
            _cache |= 1 << 40
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        await self.control_5_task.modify(self.msg_control_5)

    async def set_override_limit_switches(self, param):
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        param = param & 0x7
        _cache &= ~((0x7) << 45)
        _cache |= param << 45
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        await self.control_5_task.modify(self.msg_control_5)

    async def loop(self):
        await self.bus.loop()
