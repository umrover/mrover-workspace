'''
Reads and parses NMEA messages from the onboard GPS to provide
location data to the rover over LCM (/gps). Subscribes to
/rtcm and passes RTCM messages to the onboard gps to
acquire an RTK fix.
'''
import serial
import asyncio
import json
import numpy as np
from os import getenv
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import GPS, RTCM


class GPS_Manager():

    def __init__(self):

        # Dynamically loading config
        configPath = getenv('MROVER_CONFIG')
        configPath += "/config_gps/config.json"
        with open(configPath, "r") as read_file:
            self.config = json.load(read_file)['onboard']

        for key, val in self.config.items():
            self.__setattr__(str(key), val)

        # Mapping NMEA messages to their handlers
        self.NMEA_TAGS_MAPPER = {
            "GGA": self.gga_handler,
            "VTG": self.vtg_handler,
            "TXT": self.txt_handler
        }

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

    def gga_handler(self, msg, gps_struct):
        '''
        Handles NMEA statments that follow the **GGA pattern.
        These statements hold the lat, lon, and quality of connection
        '''
        fields = msg.split(',')

        # Handles empty messages while finding fix
        if not fields[2] or not fields[4]:
            raise ValueError('No Satelite Fix: {}'.format(msg))

        raw_lat = float(fields[2])
        lat_dir = fields[3]

        raw_lon = float(fields[4])
        lon_dir = fields[5]

        quality = fields[6]

        lat_deg = raw_lat // 100
        lat_min = (raw_lat - lat_deg * 100)

        lon_deg = raw_lon // 100
        lon_min = (raw_lon - lon_deg * 100)

        if lat_dir == 'S':
            lat_deg *= -1
            lat_min *= -1

        if lon_dir == 'W':
            lon_deg *= -1
            lon_min *= -1

        gps_struct.latitude_deg = int(lat_deg)
        gps_struct.latitude_min = lat_min
        gps_struct.longitude_deg = int(lon_deg)
        gps_struct.longitude_min = lon_min
        gps_struct.quality = np.uint8(quality)

        if quality == 0:
            print('Warning: Fix Quality Invalid')

    def vtg_handler(self, msg, gps_struct):
        '''
        Handles NMEA messages that follow the **VTG pattern.
        These messages hold the ground speed and bearing
        '''
        fields = msg.split(',')

        # degrees relative to true north
        try:
            track_made_good = float(fields[1])
        except Exception as e:
            track_made_good = float(999)

        # kilometers / hour
        try:
            speed_over_ground = float(fields[7])
        except Exception as e:
            speed_over_ground = float(0)

        gps_struct.bearing_deg = track_made_good
        gps_struct.speed = speed_over_ground * 1000 / 60

    def txt_handler(self, msg, gps_struct):
        '''
        Prints info messages revieved from GPS to screen
        '''
        print(msg)

    async def recieve(self, lcm):
        '''
        Reads from the rover GPS over serial connection.
        Attempts to read and proccess all supported NMEA tags
        at least once before publishing a new LCM message.
        Will skip publishing if the GPS quality is 0 (no fix).
        Will complain (but not crash) if encountering an
        unsupported message. Sleeps after publishing to
        allow time for handling incoming LCM messages
        '''
        gps_struct = GPS()
        error_counter = 0
        # Mark TXT as always seen because they are not necessary
        seen_tags = {tag: False if not tag == 'TXT' else True
                     for tag in self.NMEA_TAGS_MAPPER.keys()}
        while True:
            # Wait for all tags to be seen
            while (not all(seen_tags.values())):
                try:
                    msg = str(self.ser.readline())
                    print(msg)
                    error_counter = 0
                except Exception as e:
                    if error_counter < self.max_error_count:
                        error_counter += 1
                        print(e)
                        await asyncio.sleep(self.sleep)
                        continue
                    else:
                        raise e

                match_found = False
                for tag, func in self.NMEA_TAGS_MAPPER.items():
                    if tag in msg:
                        match_found = True
                        try:
                            func(msg, gps_struct)
                            seen_tags[tag] = True
                        except Exception as e:
                            print(e)
                        break

                if not match_found:
                    print('Error decoding message stream: {}'.format(msg))
                    # raise ValueError('Unrecognized NMEA string: {}'
                    #                  .format(msg))

            # Skip publish if fix is invalid
            if gps_struct.quality == 0:
                print('Waiting for GPS Fix')
                await(asyncio.sleep(self.sleep))
                continue

            lcm.publish('/gps', gps_struct.encode())
            seen_tags = {tag: False if not tag == 'TXT' else True
                         for tag in self.NMEA_TAGS_MAPPER.keys()}
            await asyncio.sleep(self.sleep)

    def transmit(self, channel, msg):
        '''
        Recieves binary RTCM messages from /rtcm LCM and
        writes them directly to the rover GPS
        '''
        struct = RTCM.decode(msg)
        print('Recieved: {} bytes'.format(len(bytes(struct.data))))
        self.ser.write(bytes(struct.data))


def main():
    # Uses a context manager to ensure serial port released
    with GPS_Manager() as manager:
        lcm = aiolcm.AsyncLCM()
        lcm.subscribe("/rtcm", manager.transmit)
        run_coroutines(lcm.loop(), manager.recieve(lcm))


if __name__ == "__main__":
    main()
