import can
import struct
import queue
from rover_common import talon_srx


class BufferedListener(can.Listener):
    def __init__(self, bus, device_id, arb_id, maxsize=0):
        super().__init__()
        self.bus = bus
        self.device_id = device_id
        self.arb_id = arb_id
        self.buffer = queue.Queue(maxsize)

    def on_message_received(self, msg):
        if msg.arbitration_id == self.arb_id:
            self.buffer.put(msg)

    def get_message(self, timeout=0.5):
        try:
            return self.buffer.get(block=True, timeout=timeout)
        except queue.Empty:
            return None


class LowLevel(object):
    def __init__(self, bus, device_id):
        self.bus = bus
        self.device_id = device_id
        self.listener = BufferedListener(bus, device_id,
                                         talon_srx.PARAM_RESPONSE | device_id)
        self.status_3_listener = BufferedListener(bus, device_id,
                                                  (talon_srx.STATUS_3 |
                                                   device_id), 1)
        self.notifier = can.Notifier(bus, [self.listener,
                                           self.status_3_listener])
        # for externalenable being false, control_1 is needed
        self.msg_control_1 = can.Message(
                    arbitration_id=talon_srx.CONTROL_1 | device_id,
                    extended_id=True,
                    data=[0 for i in range(0, 2)])
        self.control_1_task = can.send_periodic(
           can.rc['channel'], self.msg_control_1, 0.05)
        self.msg_control_5 = can.Message(
                    arbitration_id=talon_srx.CONTROL_5 | device_id,
                    extended_id=True,
                    data=[0 for i in range(0, 8)])
        self.control_5_task = can.send_periodic(
            can.rc['channel'], self.msg_control_5, 0.01)
        self.set_overide_limit_switches(
                    talon_srx.kLimitSwitchOverride.EnableFwd_EnableRev.value)

    def send_one(self, msg):
        try:
            self.bus.send(msg)
            print("Message sent on {}".format(self.bus.channel_info))
        except can.CanError:
            print("Message NOT sent")

    def set_param(self, param_id, param_val):
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
        msg = can.Message(arbitration_id=arbId,
                          data=struct.pack('<Q', frame)[:5],
                          extended_id=True)
        self.send_one(msg)
        while True:
            msg = self.listener.get_message()
            if msg.data[0] == param_id:
                return

    def read_enc_value(self):
        msg = self.status_3_listener.get_message()
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

    def set_override_brake_type(self, param):
        param = int(param) & 0x3
        frame = struct.unpack('<Q', self.msg_control_5.data)[0]
        frame &= ~(0x3 << 50)
        frame |= param << 50
        self.msg_control_5.data = struct.pack('<Q', frame)[:8]
        self.control_5_task.modify_data(self.msg_control_5)

    def set_demand(self, param, controlmode):
        param = int(param)
        H = (param >> 0x10) & 0xff
        M = (param >> 0x80) & 0xff
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
        self.control_5_task.modify_data(self.msg_control_5)

    def set_profile_slot_select(self, param):
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        if param == 0:
            _cache &= ~(1 << 40)
        else:
            _cache |= 1 << 40
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        self.control_5_task.modify_data(self.msg_control_5)

    def set_overide_limit_switches(self, param):
        _cache = struct.unpack('<Q', self.msg_control_5.data)[0]
        param = param & 0x7
        _cache &= ~((0x7) << 45)
        _cache |= param << 45
        self.msg_control_5.data = struct.pack('<Q', _cache)[:8]
        self.control_5_task.modify_data(self.msg_control_5)
