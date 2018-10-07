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
        self.evt.clear()
        return self.msg


class LowLevel(object):
    def __init__(self, iface, device_id):
        self.device_id = device_id
        self.bus = aiocan.AsyncBCM(iface)
        # param_response message buffer
        self.param_response_buffer = MessageBuffer(
                talon_srx.PARAM_RESPONSE | device_id)
        # Status 3 message buffer (encoders)
        self.status_3_buffer = SingleMessageBuffer(
                talon_srx.STATUS_3 | device_id)
        wait_for(self._setup())

    async def _setup(self):
        # subscribe to param_response messages
        await self.bus.subscribe(
                talon_srx.PARAM_RESPONSE | self.device_id,
                self.param_response_buffer, extended=True)
        # subscribe to Status 3 messages
        await self.bus.subscribe(
                talon_srx.STATUS_3 | self.device_id,
                self.status_3_buffer, extended=True)
        # for externalenable being false, control_1 is needed
        # Construct Control 1 message
        self.msg_control_1 = aiocan.Message(
                    arb_id=talon_srx.CONTROL_1 | self.device_id,
                    extended_id=True,
                    data=[0 for i in range(0, 2)])
        # Send Control 1 periodically
        self.control_1_task = await self.bus.send_periodic(
                self.msg_control_1, 0.05)
        # Construct Control 5 message
        self.msg_control_5 = aiocan.Message(
                    arb_id=talon_srx.CONTROL_5 | self.device_id,
                    extended_id=True,
                    data=[0 for i in range(0, 8)])
        # Send Control 5 periodically
        self.control_5_task = await self.bus.send_periodic(
                self.msg_control_5, 0.01)
        await self.set_override_limit_switches(
                talon_srx.kLimitSwitchOverride.EnableFwd_EnableRev.value)
        # Enable current limiting (untested)
        # await self.set_current_lim_enable(enable=True)

    # This function sends a CAN Message that sets an internal parameter for
    # this Talon.
    #
    # param_id:    from talon_srx.Param
    # param_value: value to set
    async def set_param(self, param_id, param_val):
        frame = 0
        if param_id in [
                talon_srx.Param.ProfileParamSlot0_P.value,
                talon_srx.Param.ProfileParamSlot0_I.value,
                talon_srx.Param.ProfileParamSlot0_D.value,
                talon_srx.Param.ProfileParamSlot1_P.value,
                talon_srx.Param.ProfileParamSlot1_I.value,
                talon_srx.Param.ProfileParamSlot1_D.value]:
            frame = talon_srx.float_to_fxp_10_22(param_val) & 0xFFFFFFFF
        elif param_id in [
                talon_srx.Param.ProfileParamSlot0_F.value,
                talon_srx.Param.ProfileParamSlot1_F.value]:
            param_val = min(max(param_val, -255), 255)
        else:
            frame = int(param_val)

        frame <<= 8
        frame |= param_id
        arbId = talon_srx.PARAM_SET | self.device_id
        # construct param_set message
        msg = aiocan.Message(arb_id=arbId,
                             extended_id=True,
                             data=struct.pack('<Q', frame)[:5])
        # send message
        await self.bus.send(msg)
        # wait for param_response message for confirmation
        while True:
            msg = await self.param_response_buffer.get(timeout=None)
            if msg.data[0] == param_id:
                return

    # This function sends a demand to the Talon SRX via Control 5.
    # Can be used to change the Talon's control mode or drive it
    # according to controlmode.
    #
    # param:        Passing the CAN ID of this Talon will change this
    #               talon to the specified control mode. to specify how
    #               to drive based on control mode: For example,
    #               - kThrottle: speed (-1023, 1023)
    #               - kPosition: target encoder position
    #               - kVelocity: target encoder velocity
    # controlmode:  from talon_srx.TalonControlMode
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

    # This function reads the quadrature encoder position for this talon's
    # motor. The data is updated on the bus every 100ms.
    #
    # UNITS: Encoder position is measured in encoder edges by default. If
    # config_encoder_cpr is called to configure counts per revolution,
    # encoder position is measured in floating point rotations.
    async def get_enc_pos(self):
        # get status 3 message from buffer
        msg = await self.status_3_buffer.get()
        if msg is None:
            return None
        # convert status 3 message data to bitstring
        _cache = struct.unpack('<Q', msg.data)[0]
        H = (_cache >> 0) & 0xff   # byte 0
        M = (_cache >> 8) & 0xff   # byte 1
        L = (_cache >> 16) & 0xff  # byte 2
        # raw = HML
        raw = 0
        raw |= H
        raw <<= 8
        raw |= M
        raw <<= 8
        raw |= L
        raw = talon_srx.sign_extend(raw, 24)
        return raw

    # This function reads the quadrature encoder velocity for this talon's
    # motor. The data is updated on the bus every 100ms.
    #
    # UNITS: Encoder velocity is measured in position ticks / 100ms by default.
    # If config_encoder_cpr is called to configure counts per revolution,
    # encoder velocity is measured in floating point RPM.
    async def get_enc_vel(self):
        # get status 3 message from buffer
        msg = await self.status_3_buffer.get()
        if msg is None:
            return None
        # convert status 3 message data to bitstring
        _cache = struct.unpack('<Q', msg.data)[0]
        H = (_cache >> 24) & 0xFF  # byte 3
        L = (_cache >> 32) & 0xFF  # byte 4
        # raw = HL
        raw = 0
        raw |= H
        raw <<= 8
        raw |= L
        raw = talon_srx.sign_extend(raw, 16)
        return raw

    async def set_override_brake_type(self, param):
        param = int(param) & 0x3
        frame = struct.unpack('<Q', self.msg_control_5.data)[0]
        frame &= ~(0x3 << 50)
        frame |= param << 50
        self.msg_control_5.data = struct.pack('<Q', frame)[:8]
        await self.control_5_task.modify(self.msg_control_5)

    # This function selects which closed-loop control profile to use for
    # this Talon.
    # param: profile to use (0 or 1)
    async def set_profile_slot_select(self, param):
        # convert control 5 message data to bitstring
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        if param == 0:
            _cache &= ~(1 << 40)
        else:
            _cache |= 1 << 40
        # convert bitstring into packed binary data
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        # update control 5
        await self.control_5_task.modify(self.msg_control_5)

    async def set_override_limit_switches(self, param):
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        param = param & 0x7
        _cache &= ~((0x7) << 45)
        _cache |= param << 45
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        await self.control_5_task.modify(self.msg_control_5)

    # This function controls enabling/disabling of current limiting
    # for this talon.
    async def set_current_lim_enable(self, enable):
        # convert control 5 message data to bitstring
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        if enable:
            _cache |= (0x40)
        else:
            _cache &= ~(0xC0)
        # convert bitstring into packed binary data
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        # update control 5
        await self.control_5_task.modify(self.msg_control_5)

    async def loop(self):
        await self.bus.loop()
