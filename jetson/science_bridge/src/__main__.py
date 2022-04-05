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
    NavStatus, ServoCmd, CarouselData, CarouselClosedLoopCmd, CarouselOpenLoopCmd, \
    Heater, HeaterAutoShutdown


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
        self.last_openloop_cmd = -100
        self.max_error_count = 20
        self.sleep = .01

    def __enter__(self):
        '''
        Opens a serial connection to the nucleo
        '''
        self.ser = serial.Serial(
            # port='/dev/ttyS4',
            port='/dev/ttyTHS1',
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

    def spectral_handler(self, m, spectral_struct):
        # msg format: <"$SPECTRAL,d0_1,d0_2, .... , d2_6">
        try:
            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            struct_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]
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

        # parse the spectral UART msg
        # add in relevant error handling
        # set the struct variables

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
            print(msg)
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
            enabled = False
            if (arr[1] == "1"):
                enabled = True
            print(enabled)
            struct.auto_shutdown_enabled = enabled
        except Exception as e:
            print(e)

    def heater_transmit(self, channel, msg):
        # Upon Receiving a heater on/off command, send a command for the appropriate mosfet
        print("Heater cmd callback")
        struct = Heater.decode(msg)
        message = "$Mosfet,{device},{enable},1111111"

        # Heater/Nichromes are 5-7 so(assuming 0 index) add 7 to get the appropriate mosfet
        translated_device = struct.device + 5
        message = message.format(device=translated_device,
                                 enable=int(struct.enabled))

        while(len(message) < 30):
            # Add an extra 1 for padding
            message += "1"
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))
        pass

    def heater_auto_transmit(self, channel, msg):
        # Send the nucleo a message telling if auto shutoff for heaters is off or on
        struct = HeaterAutoShutdown.decode(msg)
        message = "$AutoShutoff,{enable},1111"
        message = message.format(enable=int(struct.auto_shutdown_enabled))
        while(len(message) < 30):
            # Add an extra 1 for padding
            message += "1"
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))
        pass

    def mosfet_transmit(self, channel, msg):
        print("Mosfet callback")
        struct = MosfetCmd.decode(msg)

        message = "$Mosfet,{device},{enable},1111111"
        translated_device = struct.device
        # Translate individual pins to their respective nucleo pin
        # Laser and UV LED share pins 1 and 2
        # const int8_t rLed = 0;
        # const int8_t gLed = 1;
        # const int8_t bLed = 2;
        # const int8_t Laser = 3;
        # const int8_t UVLED = 4;
        # const int8_t whiteLED = 5;
        # const int8_t uvBulb = 6;
        # const int8_t nichWire0 = 7;
        # const int8_t nichWire1 = 8;
        # const int8_t nichWire2 = 9;
        # const int8_t ramanLaser = 10;
        # Starts from 0-2 for all the leds
        # Resets to 1 starting at Laser(3) so offset by 2
        if (translated_device > 2):
            translated_device -= 2
        message = message.format(device=translated_device,
                                 enable=int(struct.enable))
        print(message)
        while(len(message) < 30):
            message += ","
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))
        print("Mosfet Received")
        pass

    def nav_status(self, channel, msg):
        print("Received nav req")
        # Off, Done, Else

        struct = NavStatus.decode(msg)
        message = "$Mosfet,{device},{enable},11111111"
        # All Leds are 1 digit so hardcode in padding

        # Off = Blue
        if struct.nav_state_name == "Off":
            print("navstatus off")
            offmessage = message.format(device=2, enable=1) + "1"
            self.ser.write(bytes(offmessage, encoding='utf-8'))
            prev = 2
        # Done = Flashing green
        elif struct.nav_state_name == "Done":
            print("navstatus Done")
            # Flashing by turning on and off for 1 second intervals
            # Maybe change to
            for i in range(0, 6):
                self.ser.write(bytes(message.format(device=1, enable=1), encoding='utf-8'))
                time.sleep(1)
                self.ser.write(bytes(message.format(device=1, enable=0), encoding='utf-8'))
                time.sleep(1)
                prev = 1
        # Everytime else = Red
        else:
            print("navstatus else")
            messageon = message.format(device=0, enable=1)
            self.ser.write(bytes(messageon, encoding='utf-8'))
            prev = 0
        time.sleep(1)
        # Green should be in a finished state so no need to turn it off
        if (prev != 2):
            self.ser.write(bytes(message.format(device=2, enable=0),
                                 encoding='utf-8'))
        if (prev != 0):
            self.ser.write(bytes(message.format(device=0, enable=0),
                                 encoding='utf-8'))

    def servo_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        struct = ServoCmd.decode(msg)
        print("Received Servo Cmd")
        # parse data into expected format
        message = "$Servo,{angle0},{angle1},{angle2}"
        message = message.format(angle0=struct.angle0, angle1=struct.angle1, angle2=struct.angle2)
        print(message)
        while(len(message) < 30):
            message += ","
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
        while(len(message) < 30):
            message += ","
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    def carousel_closedloop_transmit(self, channel, msg):
        self.last_openloop_cmd = -100
        struct = CarouselClosedLoopCmd.decode(msg)
        print("Received Carousel (CL) Cmd")
        # parse data into expected format
        message = "$Carousel,{position}"
        message = message.format(position=struct.position)
        print(message)
        while(len(message) < 30):
            message += ","
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    async def receive(self, lcm):
        spectral = SpectralData()
        thermistor = ThermistorData()
        triad = SpectralData()
        carousel = CarouselData()
        heater = Heater()
        heater_auto = HeaterAutoShutdown()

        # Mark TXT as always seen because they are not necessary
        seen_tags = {tag: False if not tag == 'TXT' else True
                     for tag in self.NMEA_HANDLE_MAPPER.keys()}
        while True:
            # Wait for all tags to be seen
            while (not all(seen_tags.values())):
                try:
                    error_counter = 0
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
                    if tag in msg:
                        print(msg)
                        match_found = True
                        try:
                            if (tag == "SPECTRAL"):
                                self.spectral_handler(tx.decode(), spectral)
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
                                self.heater_shutoff_handler(msg, heater_auto)
                                lcm.publish('/heater_auto_shutdown_data', heater_auto.encode())
                            seen_tags[tag] = True
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
