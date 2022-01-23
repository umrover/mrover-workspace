'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
import asyncio
# import Adafruit_BBIO.UART as UART # for beaglebone use only
import numpy as np
import time
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ThermistorData, MosfetCmd, SpectralData, NavStatus, ServoCmd


class ScienceBridge():
    def __init__(self):
        # UART.setup("UART4")  # Specific to beaglebone
        # maps NMEA msgs to their handler
        # mosfet and servo only send msgs
        self.NMEA_HANDLE_MAPPER = {
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TRIAD": self.triad_handler,
            "TXT": self.txt_handler,
        }

        self.max_error_count = 20
        self.sleep = .01

    def __enter__(self):
        '''
        Opens a serial connection to the nucleo
        Not sure what the uart port on the jetson is
        '''
        self.ser = serial.Serial(
            # port='/dev/ttyS4',
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

    def spectral_handler(self, m, spectral_struct):
        try:
            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            struct_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]
            # print(arr)
            count = 1
            for var in struct_variables:
                if (not (count >= len(arr))):
                    setattr(spectral_struct, var, ((np.int16(arr[count]) << 8) | (np.int16(arr[count + 1]))))
                else:
                    setattr(spectral_struct, var, 0)
                count += 2
        except:
            pass

        # parse the spectral UART msg
        # add in relevant error handling
        # set the struct variables

    def thermistor_handler(self, msg, thermistor_struct):
        # msg format: <"$THERMISTOR,temperature">
        try:
            arr = msg.split(",")
            thermistor_struct.temp0 = float(arr[1])
            thermistor_struct.temp1 = float(arr[2])
            thermistor_struct.temp2 = float(arr[3])
        except:
            pass
        # parse the thermistor UART msg
        # adding relevant error handling
        # set the struct variables

    def triad_handler(self, m, triad_struct):
        try:
            m.split(',')
            arr = [s.strip().strip('\x00') for s in m.split(',')]
            struct_variables = ["d0_1", "d0_2", "d0_3", "d0_4", "d0_5", "d0_6",
                                "d1_1", "d1_2", "d1_3", "d1_4", "d1_5", "d1_6",
                                "d2_1", "d2_2", "d2_3", "d2_4", "d2_5", "d2_6"]
            # print(arr)
            count = 1
            for var in struct_variables:
                if (not (count >= len(arr))):
                    setattr(triad_struct, var, ((np.int16(arr[count]) << 8) | (np.int16(arr[count + 1]))))
                else:
                    setattr(triad_struct, var, 0)
                count += 2
        except:
            pass

        # parse the spectral UART msg
        # add in relevant error handling
        # set the struct variables

    def txt_handler(self, msg, struct):
        '''
        Prints info messages revieved from nucleo to screen
        '''
        print(msg)

    def repeater_handler(self, msg, struct):
        # Expected to be an empty message so nothing is done
        # Only included to fit struct
        pass

    def mosfet_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        print("Mosfet callback")
        struct = MosfetCmd.decode(msg)
        # parse data into expected format
        # Currently expects mosfet, device number, and enable bit along
        # with padding to reach 30 bytes
        message = "$Mosfet,{device},{enable},1111111"
        translated_device = struct.device
        # Translate individual pins to their respective nucleo pin
        # According to https://docs.google.com/spreadsheets/d/1x291oHOigmm7G-pxjsBUFsEbyl81ZurAz7vuSyXmgXo/edit#gid=0

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
        if(translated_device > 2):
            translated_device -= 2
        message = message.format(device=translated_device,
                                 enable=int(struct.enable))
        print(message)

        # Assumes that only single or double digit devices are used
        # No way we use 100 devices
        # Double digits have 7 + 1 + 2 + 1 + 1 + 1 + 7 = 20
        # single digits have 7 + 1 + 1 + 1 + 1 + 1 + 7 = 19, need to add one
        if(int(translated_device) < 10):
            # Add an extra 1 for padding
            message += "1"
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf8'))
        print("Mosfet Received")
        pass

    def nav_status(self, channel, msg):
        print("Received nav req")
        # Want the name of the status I guess?
        # Off, Done, Else

        struct = NavStatus.decode(msg)
        message = "$Mosfet,{device},{enable},11111111"
        # All Leds are 1 digit so hardcode in padding

        # Off = Blue
        if struct.nav_state_name == "Off":
            print("navstatus off")
            offmessage = message.format(device=2, enable=1) + "1"
            self.ser.write(bytes(offmessage, encoding='utf8'))
            prev = 2
        # Done = Flashing green
        elif struct.nav_state_name == "Done":
            print("navstatus Done")
            # Flashing by turning on and off for 1 second intervals
            # Maybe change to
            for i in range(0, 6):
                self.ser.write(bytes(message.format(device=1, enable=1), encoding='utf8'))
                time.sleep(1)
                self.ser.write(bytes(message.format(device=1, enable=0), encoding='utf8'))
                time.sleep(1)
                prev = 1
        # Everytime else = Red
        else:
            print("navstatus else")
            messageon = message.format(device=0, enable=1)
            self.ser.write(bytes(messageon, encoding='utf8'))
            prev = 0
        time.sleep(1)
        # Green should be in a finished state so no need to turn it off
        if (prev != 2):
            self.ser.write(bytes(message.format(device=2, enable=0),
                                 encoding='utf8'))
        if (prev != 0):
            self.ser.write(bytes(message.format(device=0, enable=0),
                                 encoding='utf8'))

    def servo_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        struct = ServoCmd.decode(msg)
        print("Received Servo Cmd")
        # parse data into expected format
        message = "$Servo,{angle0},{angle1},{angle2}"
        message = message.format(angle0=struct.angle0, angle1=struct.angle1, angle2=struct.angle2)
        print(len(message))
        """while(len(message) < 20):
            message += ","
        print(message)"""
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    async def receive(self, lcm):
        spectral = SpectralData()
        thermistor = ThermistorData()
        triad = SpectralData()
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
                    print(msg)

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
                        match_found = True
                        try:
                            if(tag == "SPECTRAL"):
                                self.spectral_handler(tx.decode(), spectral)
                                lcm.publish('/spectral_data', spectral.encode())
                                print('published spectral struct?')
                            if(tag == "THERMISTOR"):
                                self.thermistor_handler(msg, thermistor)
                                lcm.publish('/thermistor_data', thermistor.encode())
                            if(tag == "TRIAD"):
                                self.triad_handler(msg, triad)
                                lcm.publish('/spectral_triad_data', triad.encode())
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
        print("properly started")
        run_coroutines(_lcm.loop(), bridge.receive(_lcm))


if __name__ == "__main__":
    main()
