'''
Writes, reads and parses NMEA like messages from the onboard
science nucleo to operate the science boxes and get relevant data
'''
import serial
import asyncio
import Adafruit_BBIO.UART as UART
from numpy import int16
import lcm
import time
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import ThermistorData, MosfetCmd, RepeaterDropComplete, SpectralData
class ScienceBridge():
    def __init__(self):
        UART.setup("UART4") #  Specific to beaglebone
        # maps NMEA msgs to their handler
        # mosfet, ammonia, and pump only send msgs
        self.NMEA_HANDLE_MAPPER = {
            "SPECTRAL" : self.spectral_handler,
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
        self.ser = serial.Serial(
            port='/dev/ttyS4',
            baudrate=38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        return self
    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Closes serial connection to nucleo
        '''
        self.ser.close()
    def spectral_handler(self, msg, spectral_struct):
        print("in spec handler")
        try:
            arr = msg.split(",")
            spectral_struct.d0_1 = int16((arr[1] << 8) | arr[2])
            spectral_struct.d0_2 = int16((arr[3] << 8) | arr[4])
            spectral_struct.d0_3 = int16((arr[5] << 8) | arr[6])
            spectral_struct.d0_4 = int16((arr[7] << 8) | arr[8])
            spectral_struct.d0_5 = int16((arr[9] << 8) | arr[10])
            spectral_struct.d0_6 = int16((arr[11] << 8) | arr[12])
            spectral_struct.d1_1 = int16((arr[13] << 8) | arr[14])
            spectral_struct.d1_2 = int16((arr[15] << 8) | arr[16])
            spectral_struct.d1_3 = int16((arr[17] << 8) | arr[18])
            spectral_struct.d1_4 = int16((arr[19] << 8) | arr[20])
            spectral_struct.d1_5 = int16((arr[21] << 8) | arr[22])
            spectral_struct.d1_6 = int16((arr[23] << 8) | arr[24])
            spectral_struct.d2_1 = int16((arr[25] << 8) | arr[26])
            spectral_struct.d2_2 = int16((arr[27] << 8) | arr[28])
            spectral_struct.d2_3 = int16((arr[29] << 8) | arr[30])
            spectral_struct.d2_4 = int16((arr[31] << 8) | arr[32])
            spectral_struct.d2_5 = int16((arr[33] << 8) | arr[34])
            spectral_struct.d2_6 = int16((arr[35] << 8) | arr[36])

        except:
            pass

        # parse the spectral UART msg
        # add in relevant error handling
        # set the struct variables
        
    def thermistor_handler(self, msg, thermistor_struct):
        # msg format: <"$THERMISTOR,temperature">
        print("in therm handler")
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
        struct = RTCM.decode(msg)
        print('Recieved: {} bytes'.format(len(bytes(struct.data))))
        # parse data into expected format
        # Currently expects mosfet, device number, and enable bit along
        # with padding to reach 30 bytes
        message = "$Mosfet,{device},{enable},111111111111111111"
        message = message.format(device = struct.device, enable = int(struct.enable))
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.ser.write(bytes(message,encoding='utf8'))
        pass
    def rr_drop(self,channel,msg):
        # Struct is expected to be empty so no need for decoding
        message = "$Mosfet,{device},{enable},111111111111111111"
        # TODO set the specific device for the repeater on the firmware, placeholder 8.
        # This is always an enable
        message = message.format(device = 4, enable = 1)
        self.ser.write(bytes(message))
        # Publish to drop complete after sending the message.
        complete = RepeaterDropComplete()
        _lcm.publish('/rr_drop_complete', complete.encode())
    def nav_status(self,channel,msg):
        # Want the name of the status I guess?
        # Off, Done, Else
        struct = RTCM.decode(msg)
        # Off = Blue
        message = "$Mosfet,{device},{enable},111111111111111111"
        if struct.nav_state_name == "Off":
            message = message.format(device = 2, enable = 1)
            self.ser.write(bytes(message))
        # Done = Flashing green
        else if struct.nav_state_name == "Done":
            # Flashing by turning on and off for 1 second intervals
            for i in range (0,6):
                self.ser.write(bytes(message.format(device = 1, enable = 1)))
                await asyncio.sleep(self.sleep)
                self.ser.write(bytes(message.format(device = 1, enable = 0)))
                await asyncio.sleep(self.sleep)
        # Everytime else = Red
        else:
            message = message.format(device = 0, enable = 1)
            self.ser.write(bytes(message))
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
        print("inside recieve")
        spectral = SpectralData()
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
                            if(tag == "SPECTRAL"):
                                func(msg, spectral)
                                lcm.publish('/spectral_data', spectral.encode())
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
        _lcm = aiolcm.AsyncLCM()
        _lcm.subscribe("/mosfet_cmd", bridge.mosfet_transmit)
        _lcm.subscribe("/rr_drop_init",bridge.rr_drop)
        _lcm.subscribe("/nav_status",bridge.nav_status)
        print("properly started")
        #lcm.subscribe("/ammonia_cmd", bridge.ammonia_transmit)
        #lcm.subscribe("/pump_cmd", bridge.pump_transmit)
        run_coroutines(_lcm.loop(), bridge.recieve(_lcm))
if __name__ == "__main__":
    main()
    
