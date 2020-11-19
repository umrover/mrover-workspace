'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
import asyncio
import Adafruit_BBIO.UART as UART
#import numpy as np
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ThermistorData#, SpectralData, MosfetCmd
class ScienceBridge():
    def __init__(self):
        UART.setup("UART4") #  Specific to beaglebone
        # maps NMEA msgs to their handler
        # mosfet, ammonia, and pump only send msgs
        self.NMEA_HANDLE_MAPPER = {
           # "SPECTRAL" : self.spectral_handler,
            "THERMISTOR" : self.thermistor_handler,
            "TXT": self.txt_handler
        }
        self.NMEA_TRANSMIT_MAPPER = {
            #"MOSFET" : self.mosfet_transmit,
            #"AMMONIA" : self.ammonia_transmit,
            #"PUMP" : self.pump_transmit
        }
        self.max_error_count = 20
        self.sleep = 1
    def __enter__(self):
        '''
        Opens a serial connection to the nucleo
        Not sure what the uart port on the jetson is
        '''
        self.ser = serial.Serial("/dev/ttyS4", 115200)
        return self
    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Closes serial connection to nucleo
        '''
        self.ser.close()
    def spectral_handler(self, msg, spectral_struct):
        # parse the spectral UART msg
        # add in relevant error handling
        # set the struct variables
        pass
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
    def mosfet_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        # struct = RTCM.decode(msg)
        # print('Recieved: {} bytes'.format(len(bytes(struct.data))))
        # parse data into expected format
        # self.ser.write(bytes(struct.data))
        pass
    def ammonia_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        # struct = RTCM.decode(msg)
        # print('Recieved: {} bytes'.format(len(bytes(struct.data))))
        # parse data into expected format
        # self.ser.write(bytes(struct.data))
        pass
    def pump_transmit(self, channel, msg):
        # get cmd lcm and send to nucleo
        # struct = RTCM.decode(msg)
        # print('Recieved: {} bytes'.format(len(bytes(struct.data))))
        # parse data into expected format
        # self.ser.write(bytes(struct.data))
        pass
    async def recieve(self, lcm):
        #spectral = SpectralData()
        thermistor = ThermistorData()
         # Mark TXT as always seen because they are not necessary
        seen_tags = {tag: False if not tag == 'TXT' else True
                     for tag in self.NMEA_HANDLE_MAPPER.keys()}
        while True:
            # Wait for all tags to be seen
            while (not all(seen_tags.values())):
                try:
                    error_counter = 0
                    msg = str(self.ser.readline())
                except Exception as e:
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
                            #if(tag == "SPECTRAL"):
                                #func(msg, spectral)
                                #lcm.publish('/spectral_data', spectral.encode())
                            if(tag == "THERMISTOR"):
                                self.thermistor_handler(msg, thermistor)
                                lcm.publish('/thermistor_data', thermistor.encode())
                            seen_tags[tag] = True
                        except Exception as e:
                            print(e)
                        break
                if not match_found:
                    print('Error decoding message stream: {}'.format(msg))
            seen_tags = {tag: False if not tag == 'TXT' else True
                         for tag in self.NMEA_HANDLE_MAPPER.keys()}
            await asyncio.sleep(self.sleep)
def main():
    # Uses a context manager to ensure serial port released
    with ScienceBridge() as bridge:
        lcm = aiolcm.AsyncLCM()
        lcm.subscribe("/mosfet_cmd", bridge.mosfet_transmit)
        #lcm.subscribe("/ammonia_cmd", bridge.ammonia_transmit)
        #lcm.subscribe("/pump_cmd", bridge.pump_transmit)
        run_coroutines(lcm.loop(), bridge.recieve(lcm))
if __name__ == "__main__":
    main()
    
