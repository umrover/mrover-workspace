import sys
import can
import struct
import time
from can.interfaces.interface import Bus
from . import talon_srx


device_id = None
encoder_vel = 0
encoder_pos = 0


control_types = {
    talon_srx.CONTROL_1: 'CONTROL_1',
    talon_srx.CONTROL_2: 'CONTROL_2',
    talon_srx.CONTROL_3: 'CONTROL_3',
    talon_srx.CONTROL_5: 'CONTROL_5',
    talon_srx.CONTROL_6: 'CONTROL_6',
    talon_srx.PARAM_SET: 'PARAM_SET',
    talon_srx.PARAM_REQUEST: 'PARAM_REQUEST',
    talon_srx.PARAM_RESPONSE: 'PARAM_RESPONSE'
}


def process_param_set_msg(msg):
    param = talon_srx.Param(msg.data[0])
    raw_bits = struct.unpack('<I', msg.data[1:5])[0]
    val = raw_bits
    if param in (
            talon_srx.Param.ProfileParamSlot0_P,
            talon_srx.Param.ProfileParamSlot0_I,
            talon_srx.Param.ProfileParamSlot0_D,
            talon_srx.Param.ProfileParamSlot1_P,
            talon_srx.Param.ProfileParamSlot1_I,
            talon_srx.Param.ProfileParamSlot1_D):
        val = talon_srx.fxp_10_22_to_float(raw_bits)
    print('setting parameter {} to {}'.format(
        param.name, val))
    # TODO send a paramresponse message?


msg_processors = {
    talon_srx.PARAM_SET: process_param_set_msg
}


class Listener(can.Listener):
    def on_message_received(self, msg):
        incoming_dev_id = msg.arbitration_id & (~talon_srx.BITMASK)
        if incoming_dev_id == device_id:
            control_type = msg.arbitration_id & talon_srx.BITMASK
            if control_type in control_types:
                print('received {} messsage: {}'.format(
                    control_types[control_type], msg))
                if control_type in msg_processors:
                    msg_processors[control_type](msg)
            elif (control_type >= talon_srx.STATUS_1 and
                    control_type <= (talon_srx.STATUS_11 | 63)):
                # Ignore status messages
                pass
            else:
                print('unknown message')


def make_status_3_msg():
    one_byte_bitmask = 0xFF
    # TODO check this later
    # encoder position
    msg_data = [
        ((encoder_pos >> 16) & one_byte_bitmask),
        ((encoder_pos >> 8) & one_byte_bitmask),
        (encoder_pos & one_byte_bitmask)
    ]
    # encoder velocity
    msg_data.extend([
        ((encoder_vel >> 8) & one_byte_bitmask),
        (encoder_vel & one_byte_bitmask)
    ])

    # The rest of the message should not be necessary for our code, so we'll
    # just fill the remainder of the message with zero bytes
    msg_data.extend([0]*3)
    return msg_data


def usage():
    print('usage: {} ID'.format(sys.argv[0]), file=sys.stderr)
    print('\twhere 0 <= ID < 63', file=sys.stderr)


def main():
    global device_id
    global encoder_value_send_task
    if len(sys.argv) != 2:
        usage()
        sys.exit(1)

    device_id = int(sys.argv[1])
    if device_id < 0 or device_id >= 63:
        usage()
        sys.exit(1)

    can.rc['interface'] = 'socketcan_ctypes'
    can.rc['channel'] = 'vcan0'

    bus = Bus()
    notifier = can.Notifier(bus, [Listener()])
    status_3_msg = can.Message(
            arbitration_id=(talon_srx.STATUS_3 | device_id),
            extended_id=True,
            data=make_status_3_msg())
    status_3_task = can.send_periodic(
            can.rc['channel'], status_3_msg, 0.1)
    while True:
        status_3_msg.data = make_status_3_msg()
        status_3_task.modify_data(status_3_msg)
        time.sleep(0.05)
    notifier.stop()
