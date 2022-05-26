'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
import asyncio
# import Adafruit_BBIO.UART as UART  # for beaglebone use only
import numpy as np
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ThermistorData, MosfetCmd, SpectralData, \
    AutonLed, ServoCmd, Heater, HeaterAutoShutdown
from enum import Enum


class LED_state(Enum):
    RED = 0
    GREEN = 1
    BLUE = 2
    NONE = 3


# Mapping of LCM mosfet devices to actual mosfet devices
class Mosfet_devices(Enum):
    RED_LED = 10
    GREEN_LED = 4
    BLUE_LED = 5
    RA_LASER = 1
    UV_LED = 4
    WHITE_LED = 5
    UV_BULB = 6
    RAMAN_LASER = 10


Mosfet_Map = [Mosfet_devices.RED_LED.value, Mosfet_devices.GREEN_LED.value,
              Mosfet_devices.BLUE_LED.value, Mosfet_devices.RA_LASER.value,
              Mosfet_devices.UV_LED.value, Mosfet_devices.WHITE_LED.value,
              Mosfet_devices.UV_BULB.value, Mosfet_devices.RAMAN_LASER.value]


# Mapping of Heater devices to actual mosfet devices
class Heater_devices(Enum):
    HEATER_0 = 7
    HEATER_1 = 8
    HEATER_2 = 9


Heater_Map = [Heater_devices.HEATER_0.value, Heater_devices.HEATER_1.value, Heater_devices.HEATER_2.value]


UART_TRANSMIT_MSG_LEN = 30


class ScienceBridge():
    def __init__(self):
        # UART.setup("UART4")  # Specific to beaglebone
        # maps NMEA msgs to their handler
        self.NMEA_HANDLE_MAPPER = {
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler,
            "TXT": self.txt_handler,
            "HEATER": self.heater_state_handler,
            "AUTOSHUTOFF": self.heater_shutoff_handler
        }
        self.max_error_count = 20
        self.sleep = .01
        self.previous_auton_msg = "Default"

        self.led_map = {
            "Red": LED_state.RED,
            "Blue": LED_state.BLUE,
            "Green": LED_state.GREEN
        }

    def __enter__(self):
        '''
        Opens a serial connection to the nucleo
        '''
        self.ser = serial.Serial(
            # port='/dev/ttyS4',
            # port='/dev/ttyTHS1',  # used on science nano
            port='/dev/ttyTHS0',
            baudrate=38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Closes serial connection to nucleo
        '''
        self.ser.close()

    def add_padding(self, msg):
        while(len(msg) < UART_TRANSMIT_MSG_LEN):
            msg += ","
        return msg

    def uart_send(self, message):
        message = self.add_padding(message)
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    def format_mosfet_message(self, device, enable):
        mosfet_message = "$MOSFET,{dev},{en},"
        return mosfet_message.format(dev=device, en=enable)

    def spectral_handler(self, m, spectral_struct):
        # msg format: <"$SPECTRAL,d0_1,d0_2, .... , d2_6">
        try:
            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            struct_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]

            # There are 3 spectral sensors, each having 6 channels.
            # We read a uint16_t from each channel.
            # The jetson reads byte by byte, so the program combines every two byte of information
            # into a uint16_t.
            count = 1
            for var in struct_variables:
                if (not (count >= len(arr))):
                    setattr(spectral_struct, var, 0xFFFF & ((np.uint8(arr[count]) << 8) | (np.uint8(arr[count + 1]))))
                else:
                    setattr(spectral_struct, var, 0)
                count += 2
        except Exception as e:
            print("spectral exception:", e)
            pass

        # parse the spectral UART msg
        # add in relevant error handling
        # set the struct variables

    def thermistor_handler(self, msg, thermistor_struct):
        # msg format: <"$THERMISTOR,temp0,temp1,temp2">
        try:
            arr = msg.split(",")
            thermistor_struct.temp0 = float(arr[1])
            thermistor_struct.temp1 = float(arr[2])
            thermistor_struct.temp2 = float(arr[3])
        except Exception as e:
            print("thermistor exception:", e)
            pass
        # parse the thermistor UART msg
        # adding relevant error handling
        # set the struct variables

    def triad_handler(self, m, triad_struct):
        # msg format: <"$TRIAD,d0_1,d0_2, .... , d2_6">
        try:

            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            struct_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]

            # There are 18 channels.
            # We read a uint16_t from each channel.
            # The jetson reads byte by byte, so the program combines every two byte of information
            # into a uint16_t.
            count = 1
            for var in struct_variables:
                if (not (count >= len(arr))):
                    pass
                    setattr(triad_struct, var, 0xFFFF & ((np.uint8(arr[count+1]) << 8) | (np.uint8(arr[count]))))
                else:
                    setattr(triad_struct, var, 0)
                count += 2
        except Exception as e:
            print("triad exception:", e)
            pass

    def txt_handler(self, msg, struct):
        '''
        Prints info messages revieved from nucleo to screen
        '''
        print(msg)

    def heater_state_handler(self, msg, struct):
        # msg format: <"$HEATER,device,enabled">
        try:
            arr = msg.split(",")
            # Send back the heater and the state
            struct.device = int(arr[1])
            struct.enabled = bool(int(arr[2]))
        except Exception as e:
            print(e)

    def heater_shutoff_handler(self, msg, struct):
        # msg format: <"$AUTOSHUTOFF,device,enabled">
        try:
            arr = msg.split(",")
            enabled = bool(int(arr[1]))
            struct.auto_shutdown_enabled = enabled
        except Exception as e:
            print(e)

    def heater_transmit(self, channel, msg):
        # Upon Receiving a heater on/off command, send a command for the appropriate mosfet
        print("Heater cmd callback")
        struct = Heater.decode(msg)

        translated_device = Heater_Map[struct.device]
        message = self.format_mosfet_message(translated_device, int(struct.enabled))
        self.uart_send(message)

    def heater_auto_transmit(self, channel, msg):
        # Send the nucleo a message telling if auto shutoff for heaters is off or on
        struct = HeaterAutoShutdown.decode(msg)
        message = "$AUTOSHUTOFF,{enable}"
        message = message.format(enable=int(struct.auto_shutdown_enabled))
        self.uart_send(message)

    def mosfet_transmit(self, channel, msg):
        print("Mosfet callback")
        struct = MosfetCmd.decode(msg)

        translated_device = Mosfet_Map[struct.device]
        message = self.format_mosfet_message(translated_device, int(struct.enable))
        self.uart_send(message)
        print("Mosfet Received")
        pass

    def auton_led(self, channel, msg):  # TODO - fix make better
        # Off, Done, On
        struct = AutonLed.decode(msg)
        try:
            requested_state = self.led_map[struct.color]
            print("Received new auton led request: Turning " + str(struct.color))
        except KeyError:
            requested_state = LED_state.NONE
            print("Received invalid/Off auton led request: Turning OFF all colors")

        led_message = "$LED,{led_color}".format(led_color=requested_state.value)
        self.uart_send(led_message)

    def servo_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        struct = ServoCmd.decode(msg)
        print("Received Servo Cmd")
        # parse data into expected format
        message = "$SERVO,{angle0},{angle1},{angle2}"
        message = message.format(angle0=struct.angle0, angle1=struct.angle1, angle2=struct.angle2)
        self.uart_send(message)

    async def receive(self, lcm):
        spectral = SpectralData()
        thermistor = ThermistorData()
        triad = SpectralData()
        heater = Heater()
        heater_auto_shutdown = HeaterAutoShutdown()

        # Mark TXT as always seen because they are not necessary
        seen_tags = {tag: False if not tag == 'TXT' else True
                     for tag in self.NMEA_HANDLE_MAPPER.keys()}
        while True:
            # Wait for all tags to be seen
            while (not all(seen_tags.values())):  # TODO -
                try:
                    error_counter = 0  # TODO - DELETE
                    tx = self.ser.readline()
                    msg = str(tx)
                except Exception as e:
                    print("Errored")
                    if error_counter < self.max_error_count:
                        error_counter += 1
                        print(e)
                        await asyncio.sleep(self.sleep)
                        continue
                    else:
                        raise e
                match_found = False
                for tag, func in self.NMEA_HANDLE_MAPPER.items():
                    if tag in msg:  # TODO - why do we have tag in msg, func is not even used
                        print(msg)
                        match_found = True
                        try:
                            if (tag == "SPECTRAL"):
                                self.spectral_handler(tx.decode(), spectral)  # TODO - why is this tx.decode()
                                lcm.publish('/spectral_data', spectral.encode())
                            elif (tag == "THERMISTOR"):
                                self.thermistor_handler(msg, thermistor)
                                lcm.publish('/thermistor_data', thermistor.encode())
                            elif (tag == "TRIAD"):
                                self.triad_handler(msg, triad)
                                lcm.publish('/spectral_triad_data', triad.encode())
                            elif (tag == "HEATER"):
                                self.heater_state_handler(msg, heater)
                                lcm.publish('/heater_state_data', heater.encode())
                            elif (tag == "AUTOSHUTOFF"):
                                self.heater_shutoff_handler(msg, heater_auto_shutdown)
                                lcm.publish('/heater_auto_shutdown_data', heater_auto_shutdown.encode())
                            seen_tags[tag] = True  # TODO - move to top so not hidden, or just don't use.
                        except Exception as e:
                            print(e)
                        break
                if not match_found:
                    if not msg:
                        print('Error decoding message stream: {}'.format(msg))
                await asyncio.sleep(self.sleep)
            seen_tags = {tag: False if not tag == 'TXT' else True
                         for tag in self.NMEA_HANDLE_MAPPER.keys()}
            await asyncio.sleep(self.sleep)


def main():
    # Uses a context manager to ensure serial port released
    with ScienceBridge() as bridge:
        _lcm = aiolcm.AsyncLCM()
        _lcm.subscribe("/mosfet_cmd", bridge.mosfet_transmit)
        _lcm.subscribe("/auton_led", bridge.auton_led)
        _lcm.subscribe("/servo_cmd", bridge.servo_transmit)
        _lcm.subscribe("/heater_cmd", bridge.heater_transmit)
        _lcm.subscribe("/heater_auto_shutdown_cmd", bridge.heater_auto_transmit)
        print("properly started")
        run_coroutines(_lcm.loop(), bridge.receive(_lcm))


if __name__ == "__main__":
    main()
