'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
import asyncio
# import Adafruit_BBIO.UART as UART  # for beaglebone use only
import numpy as np
import time
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ThermistorData, MosfetCmd, SpectralData, \
    NavStatus, ServoCmd, CarouselPosition, CarouselOpenLoopCmd, \
    Heater, HeaterAutoShutdown
from enum import Enum


class Auton_state(Enum):
    ELSE = 0
    DONE = 1
    OFF = 2


# Mapping of LCM mosfet devices to actual mosfet devices
class Mosfet_devices(Enum):
    RED_LED = 10
    GREEN_LED = 4
    BLUE_LED = 5
    RA_LASER = 10
    UV_LED = 4
    WHITE_LED = 5
    UV_BULB = 6
    RAMAN_LASER = 10


Mosfet_Map = [Mosfet_devices.RED_LED, Mosfet_devices.GREEN_LED,
              Mosfet_devices.BLUE_LED, Mosfet_devices.RA_LASER,
              Mosfet_devices.UV_LED, Mosfet_devices.WHITE_LED,
              Mosfet_devices.UV_BULB, Mosfet_devices.RAMAN_LASER]


# Mapping of Heater devices to actual mosfet devices
class Heater_devices(Enum):
    HEATER_0 = 7
    HEATER_1 = 8
    HEATER_2 = 9


Heater_Map = [Heater_devices.HEATER_0, Heater_devices.HEATER_1, Heater_devices.HEATER_2]


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
            "CAROUSEL": self.carousel_handler,
            "HEATER": self.heater_state_handler,
            "AUTOSHUTOFF": self.heater_shutoff_handler
        }
        self.last_openloop_cmd = -1
        self.max_error_count = 20
        self.sleep = .01

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
            count = 0
            for var in struct_variables:
                if (not (count > len(arr))):
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
            count = 0
            for var in struct_variables:
                if (not (count > len(arr))):
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

    def carousel_handler(self, msg, carousel_struct):
        # msg format: <"$CAROUSEL,position>
        try:
            arr = msg.split(",")
            carousel_struct.position = np.int8(arr[1])
        except Exception as e:
            print(e)

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
        # msg format: <"$auto_shutoff,device,enabled">
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
        message = "$Mosfet,{device},{enable},"

        # Heater/Nichromes mosfet devices 7, 8, and 9, so add 7 to get the appropriate mosfet

        translated_device = Heater_Map[struct.device]
        message = message.format(device=translated_device,
                                 enable=int(struct.enabled))

        while(len(message) < UART_TRANSMIT_MSG_LEN):
            # Add extra , for padding
            message += ","
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))
        pass

    def heater_auto_transmit(self, channel, msg):
        # Send the nucleo a message telling if auto shutoff for heaters is off or on
        struct = HeaterAutoShutdown.decode(msg)
        message = "$AutoShutoff,{enable}"
        message = message.format(enable=int(struct.auto_shutdown_enabled))
        while(len(message) < UART_TRANSMIT_MSG_LEN):
            # add extra , for padding
            message += ","
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))
        pass

    def mosfet_transmit(self, channel, msg):
        print("Mosfet callback")
        struct = MosfetCmd.decode(msg)

        message = "$Mosfet,{device},{enable}"
        translated_device = Mosfet_Map[struct.device]

        message = message.format(device=translated_device,
                                 enable=int(struct.enable))
        print(message)
        message = self.add_padding(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))
        print("Mosfet Received")
        pass

    def nav_status(self, channel, msg):  # TODO - fix make better
        print("Received nav req")
        # Off, Done, Else

        struct = NavStatus.decode(msg)
        message = "$Mosfet,{device},{enable}" 

        # Off = Blue
        if struct.nav_state_name == "Off":
            print("navstatus off")
            blue = message.format(device=Mosfet_devices.BLUE_LED, enable=1)
            blue = self.add_padding(blue)
            self.ser.write(bytes(blue, encoding='utf-8'))
            prev = Auton_state.OFF
        # Done = Flashing green
        elif struct.nav_state_name == "Done":
            print("navstatus Done")
            # Flashing by turning on and off for 1 second intervals

            NUMBER_OF_LED_BLINKS = 6

            for i in range(NUMBER_OF_LED_BLINKS):
                green_on = message.format(device=Mosfet_devices.GREEN_LED, 
                                          enable=1)
                green_on = self.add_padding(green_on)
                self.ser.write(bytes(green_on, encoding='utf-8'))
                time.sleep(1)
                green_off = message.format(device=Mosfet_devices.GREEN_LED, 
                                           enable=0)
                green_off = self.add_padding(green_off)
                self.ser.write(bytes(green_off, encoding='utf-8'))
                time.sleep(1)
                prev = Auton_state.DONE
        # Everytime else = Red
        else:
            print("navstatus else")
            red = message.format(device=Mosfet_devices.RED_LED, enable=1)
            red = self.add_padding(red)
            self.ser.write(bytes(red, encoding='utf-8'))
            prev = Auton_state.ELSE
        time.sleep(1)
        # Green should be in a finished state so no need to turn it off
        if (prev != Auton_state.OFF):
            green_off = message.format(device=Mosfet_devices.GREEN_LED, enable=0)
            green_off = self.add_padding(green_off)
            self.ser.write(bytes(green_off, encoding='utf-8'))
        if (prev != Auton_state.ELSE):
            red_off = message.format(device=Mosfet_devices.RED_LED, enable=0)
            red_off = self.add_padding(red_off)
            self.ser.write(bytes(red_off, encoding='utf-8'))

    def servo_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        struct = ServoCmd.decode(msg)
        print("Received Servo Cmd")
        # parse data into expected format
        message = "$Servo,{angle0},{angle1},{angle2}"
        message = message.format(angle0=struct.angle0, angle1=struct.angle1, angle2=struct.angle2)
        print(message)
        message = self.add_padding(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    def carousel_openloop_transmit(self, channel, msg):
        struct = CarouselOpenLoopCmd.decode(msg)

        if (self.last_openloop_cmd == struct.throttle):
            return
        print("Received Carousel (OL) Cmd")
        self.last_openloop_cmd = struct.throttle
        # parse data into expected format
        message = "$OpenCarousel,{throttle}"
        message = message.format(throttle=struct.throttle)
        print(message)
        while(len(message) < UART_TRANSMIT_MSG_LEN):
            message += ","
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    def carousel_closedloop_transmit(self, channel, msg):
        self.last_openloop_cmd = -1
        struct = CarouselPosition.decode(msg)
        print("Received Carousel (CL) Cmd")
        # parse data into expected format
        message = "$Carousel,{position}"
        message = message.format(position=struct.position)
        print(message)
        while(len(message) < UART_TRANSMIT_MSG_LEN):
            message += ","
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    async def receive(self, lcm):
        spectral = SpectralData()
        thermistor = ThermistorData()
        triad = SpectralData()
        carousel = CarouselPosition()
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
                            elif (tag == "CAROUSEL"):
                                self.carousel_handler(msg, carousel)
                                lcm.publish('/carousel_data', carousel.encode())
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
        _lcm.subscribe("/nav_status", bridge.nav_status)
        _lcm.subscribe("/servo_cmd", bridge.servo_transmit)
        _lcm.subscribe("/carousel_openloop_cmd", bridge.carousel_openloop_transmit)
        _lcm.subscribe("/carousel_closedloop_cmd", bridge.carousel_closedloop_transmit)
        _lcm.subscribe("/heater_cmd", bridge.heater_transmit)
        _lcm.subscribe("/heater_auto_shutdown_cmd", bridge.heater_auto_transmit)
        print("properly started")
        run_coroutines(_lcm.loop(), bridge.receive(_lcm))


if __name__ == "__main__":
    main()
