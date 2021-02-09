'''
Reads RTCM messages from the base station gps and sends them
over LCM (/rtcm) to the onboard gps.
'''
import serial
import time
import json
from os import getenv
from rover_common import aiolcm
from rover_msgs import RTCM


class GPS_Base_Station():

    def __init__(self):

        # Dynamically loading config
        configPath = getenv('MROVER_CONFIG')
        configPath += "/config_gps/config.json"
        with open(configPath, "r") as read_file:
            self.config = json.load(read_file)['base_station']

        for key, val in self.config.items():
            self.__setattr__(str(key), val)

    def __enter__(self):
        '''
        Opens a serial connection to the GPS
        '''
        self.ser = serial.Serial(self.filename, self.baudrate)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        '''
        Closes serial connection to GPS
        '''
        self.ser.close()

    def run(self):
        '''
        Reads a chunk of serial data from the base station GPS
        and sends it over LCM to the rover GPS. The RTCM messages
        are not being interpreted, merely chunked into bytes so they
        can be packaged up.
        '''
        lcm = aiolcm.AsyncLCM()
        rtcm = RTCM()
        while (self.ser.is_open):
            try:
                byte_string = self.ser.read(self.buffsize)
                print(byte_string)
            except Exception as e:
                print(e)
                time.sleep(1)
                continue
            rtcm.size = len(byte_string)
            rtcm.data = list(byte_string)
            lcm.publish('/rtcm', rtcm.encode())


def main():
    # Use of a context manager ensures proper release of serial port
    with GPS_Base_Station() as base_station:
        base_station.run()


if __name__ == "__main__":
    main()
