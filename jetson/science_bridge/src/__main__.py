'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
import asyncio
import Adafruit_BBIO.UART as UART
import numpy as np
# import re
import time
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ThermistorData, MosfetCmd, RepeaterDrop, SpectralData, NavStatus, AmmoniaCmd


class ScienceBridge():
    def __init__(self):
        UART.setup("UART4")  # Specific to beaglebone
        # maps NMEA msgs to their handler
        # mosfet, ammonia, and pump only send msgs
        self.NMEA_HANDLE_MAPPER = {
            "SPECTRAL": self.spectral_handler,
            "THERMISTOR": self.thermistor_handler,
            "TXT": self.txt_handler,
            "REPEATER": self.repeater_handler
        }
        self.NMEA_TRANSMIT_MAPPER = {
            # "MOSFET" : self.mosfet_transmit,
            # "AMMONIA" : self.ammonia_transmit,
            # "PUMP" : self.pump_transmit
        }
        self.max_error_count = 20
        self.sleep = .01

    def __enter__(self):
        '''
        Opens a serial connection to the nucleo
        Not sure what the uart port on the jetson is
        '''
        self.ser = serial.Serial(
            port='/dev/ttyS4',
            # port='/dev/ttyTHS0',
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
        message = message.format(device=struct.device,
                                 enable=int(struct.enable))
        print(message)

        # Assumes that only single or double digit devices are used
        # No way we use 100 devices
        # Double digits have 7 + 1 + 2 + 1 + 1 + 1 + 7 = 20
        # single digits have 7 + 1 + 1 + 1 + 1 + 1 + 7 = 19, need to add one
        if(int(struct.device) < 10):
            # Add an extra 1 for padding
            message += "1"
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf8'))
        print("Mosfet Received")
        pass

    def rr_drop(self, channel, msg):
        print("Received rr_drop req")
        # Struct is expected to be empty so no need for decoding
        message = "$Mosfet,{device},{enable},11111111"
        # This is always an enable
        # Should be tied to SA UV = 4
        # Always a single digit so no need to check for double
        message = message.format(device=4, enable=1)
        self.ser.write(bytes(message, encoding='utf8'))
        # Publish to drop complete after sending the message.

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

    def ammonia_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        struct = AmmoniaCmd.decode(msg)
        print("Received Ammonia Cmd")
        # parse data into expected format
        # self.ser.write(bytes(struct.data))
        message = "$AMMONIA,{speed}"
        message = message.format(speed=struct.speed)
        print(len(message))
        while(len(message) < 20):
            message += ","
        print(message)
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message, encoding='utf-8'))

    # #if ser.isOpen():
    # while(1):
    #     print("Serial is open!")
    #     ser.write("$AMMONIA,0,,,".encode('utf-8'))
    #     time.sleep(1)
    # ser.close()
    # #

    async def recieve(self, lcm):
        spectral = SpectralData()
        thermistor = ThermistorData()
        rr_drop = RepeaterDrop()
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
                            if(tag == "REPEATER"):
                                # Empty message so no handler required.
                                lcm.publish('/rr_drop_complete', rr_drop.encode())
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
        _lcm.subscribe("/rr_drop_init", bridge.rr_drop)
        _lcm.subscribe("/nav_status", bridge.nav_status)
        _lcm.subscribe("/ammonia_cmd", bridge.ammonia_transmit)
        print("properly started")
        # lcm.subscribe("/pump_cmd", bridge.pump_transmit)
        run_coroutines(_lcm.loop(), bridge.recieve(_lcm))


if __name__ == "__main__":
    main()
