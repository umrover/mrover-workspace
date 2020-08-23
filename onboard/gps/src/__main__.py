import serial
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_common import aiolcm
from rover_msgs import GPS, RTCM

# To be moved to config
_BAUDRATE = 115200
_FILENAME = '/dev/ttyACM1'
_MAX_ERROR_COUNT = 5
_SLEEP = 0.1


class GPS_Manager():

    def __init__(self):
        self.NMEA_TAGS_MAPPER = {
            "GGA": self.gga_handler,
            "VTG": self.vtg_handler
        }

    def __enter__(self):
        self.ser = serial.Serial(_FILENAME, _BAUDRATE)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.ser.close()

    def gga_handler(self, msg, gps_struct):
        fields = msg.split(',')

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
        gps_struct.quality = int(quality)

        if quality == 0:
            print('Warning: Fix Quality Invalid')

    def vtg_handler(self, msg, gps_struct):
        fields = msg.split(',')

        # degrees relative to true north
        try:
            track_made_good = float(fields[1])
        except Exception as e:
            # print(e)
            track_made_good = float(999)

        # kilometers / hour
        try:
            speed_over_ground = float(fields[7])
        except Exception as e:
            # print(e)
            speed_over_ground = float(0)

        gps_struct.bearing_deg = track_made_good
        gps_struct.speed = speed_over_ground

    async def recieve(self, lcm):
        gps_struct = GPS()
        error_counter = 0
        seen_tags = {tag: False for tag in self.NMEA_TAGS_MAPPER.keys()}
        while True:
            # Wait for all tags to be seen
            while (not all(seen_tags.values())):
                try:
                    msg = str(self.ser.readline())
                    error_counter = 0
                except Exception as e:
                    if error_counter < _MAX_ERROR_COUNT:
                        error_counter += 1
                        print(e)
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
                    print('Error decoding message stream')
                    raise ValueError('Unrecognized NMEA string: {}'
                                     .format(msg))

            # Skip publish if fix is invalid
            if gps_struct.quality == 0:
                print('Waiting for GPS Fix')
                await(asyncio.sleep(_SLEEP))
                continue

            lcm.publish('/gps', gps_struct.encode())
            seen_tags = {tag: False for tag in self.NMEA_TAGS_MAPPER.keys()}
            await asyncio.sleep(_SLEEP)

    def transmit(self, channel, msg):
        struct = RTCM.decode(msg)
        print(self.ser.write(struct.msg.encode('ascii')))


def main():
    with GPS_Manager() as manager:
        lcm = aiolcm.AsyncLCM()
        lcm.subscribe("/rtcm", manager.transmit)
        run_coroutines(lcm.loop(), manager.recieve(lcm))


if __name__ == "__main__":
    main()
