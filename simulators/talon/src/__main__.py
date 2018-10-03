import sys
import asyncio
import struct
import os
from rover_common.aiohelper import exec_later, run_coroutines, wait_for
from rover_common import talon_srx, aiocan


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


def process_param_set_msg(bus, msg, device_id):
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

    msg = aiocan.Message(
            arb_id=(talon_srx.PARAM_RESPONSE | device_id),
            extended_id=True,
            data=msg.data)
    exec_later(bus.send(msg))


msg_processors = {
    talon_srx.PARAM_SET: process_param_set_msg
}


class Listener:
    def __init__(self, bus, device_id):
        super().__init__()
        self.bus = bus
        self.device_id = device_id

    def __call__(self, msg):
        incoming_dev_id = msg.arb_id & (~talon_srx.BITMASK)
        if incoming_dev_id == self.device_id:
            control_type = msg.arb_id & talon_srx.BITMASK
            if control_type in control_types:
                print('received {} messsage: {}'.format(
                    control_types[control_type], msg))
                if control_type in msg_processors:
                    msg_processors[control_type](self.bus, msg, self.device_id)
            elif (control_type >= talon_srx.STATUS_1 and
                    control_type <= (talon_srx.STATUS_11 | 63)):
                # Ignore status messages
                pass
            else:
                print('unknown message')


def make_status_3_msg():
    one_byte_bitmask = 0xFF
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


async def status_3(bus, device_id):
    status_3_msg = aiocan.Message(
            arb_id=(talon_srx.STATUS_3 | device_id),
            extended_id=True,
            data=make_status_3_msg())
    status_3_task = await bus.send_periodic(status_3_msg, 0.1)
    while True:
        status_3_msg.data = make_status_3_msg()
        await status_3_task.modify(status_3_msg)
        await asyncio.sleep(0.05)


async def setup_bus(bus, device_id):
    l = Listener(bus, device_id)
    await bus.subscribe(talon_srx.CONTROL_1 | device_id, l, extended=True)
    await bus.subscribe(talon_srx.CONTROL_2 | device_id, l, extended=True)
    await bus.subscribe(talon_srx.CONTROL_3 | device_id, l, extended=True)
    await bus.subscribe(talon_srx.CONTROL_5 | device_id, l, extended=True)
    await bus.subscribe(talon_srx.CONTROL_6 | device_id, l, extended=True)
    await bus.subscribe(talon_srx.PARAM_SET | device_id, l, extended=True)
    await bus.subscribe(talon_srx.PARAM_REQUEST | device_id, l, extended=True)
    await bus.subscribe(talon_srx.PARAM_RESPONSE | device_id, l, extended=True)


def main():
    global encoder_value_send_task
    if len(sys.argv) != 2:
        usage()
        sys.exit(1)

    device_id = int(sys.argv[1])
    if device_id < 0 or device_id >= 63:
        usage()
        sys.exit(1)

    iface = os.environ.get('MROVER_TALON_CAN_IFACE', 'vcan0')

    bus = aiocan.AsyncBCM(iface)
    wait_for(setup_bus(bus, device_id))
    run_coroutines(bus.loop(), status_3(bus, device_id))
