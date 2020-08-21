import serial
from rover_common import aiolcm
from rover_msgs import GPS

# To be moved to config
_BAUDRATE = 38400
_FILENAME = '/dev/ttyACM0' 
_TIMEOUT = 1

def main():
    # lcm = aiolcm.AsyncLCM()
    # serialPort = serial.Serial('/dev/ttyACM0')
    # serialPort.baudrate = 9600
    # serialPort.bytesize = serial.EIGHTBITS
    # serialPort.parity = serial.PARITY_NONE
    # serialPort.stopbits = serial.STOPBITS_ONE
    with serial.Serial(_FILENAME, _BAUDRATE) as s:
        while (s.is_open):
            try:
                line = s.readline()
            except Exception as e:
                print(e)
                continue
            print(line)

            # latitude = float(serialPort.read_until(b',')[:-1])
            # ns = serialPort.read_until(b',')[:-1]
            # latitude = latitude * -1 if ns is b'S' else latitude
            # longitude = float(serialPort.read_until(b',')[:-1])
            # ew = serialPort.read_until(b',')[:-1]
            # longitude = longitude * -1 if ew is b'W' else longitude
            # speed = float(serialPort.read_until(b',')[:-1])
            # bearing = float(serialPort.read_until(b',')[:-1])

            # gps.latitude_deg = int(latitude/100)
            # gps.longitude_deg = int(longitude/100)
            # gps.latitude_min = latitude - (
            #     gps.latitude_deg * 100)
            # gps.longitude_min = longitude - (
            #     gps.longitude_deg * 100)
            # gps.bearing_deg = bearing
            # gps.speed = speed
            # lcm.publish('/gps', gps.encode())


if __name__ == "__main__":
    main()
