import serial
from rover_common import aiolcm
from rover_msgs import GPS


def main():
    lcm = aiolcm.AsyncLCM()
    serialPort = serial.Serial('/dev/ttyUSB0')
    serialPort.baudrate = 4800
    serialPort.bytesize = serial.EIGHTBITS
    serialPort.parity = serial.PARITY_NONE
    serialPort.stopbits = serial.STOPBITS_ONE

    while (True):
        oneByte = serialPort.read()
        if oneByte != b'$':
            continue

        fiveBytes = serialPort.read(5)
        if fiveBytes != b'GPRMC':
            continue

        gps = GPS()
        serialPort.read_until(b',')[:-1]
        serialPort.read_until(b',')[:-1]

        hasFix = serialPort.read_until(b',')[:-1]
        if hasFix != b'A':
            continue

        latitude = float(serialPort.read_until(b',')[:-1])
        ns = serialPort.read_until(b',')[:-1]
        latitude = latitude * -1 if ns is b'S' else latitude
        longitude = float(serialPort.read_until(b',')[:-1])
        ew = serialPort.read_until(b',')[:-1]
        longitude = longitude * -1 if ew is b'W' else longitude
        speed = float(serialPort.read_until(b',')[:-1])
        bearing = float(serialPort.read_until(b',')[:-1])

        gps.latitude_deg = int(latitude/100)
        gps.longitude_deg = int(longitude/100)
        gps.latitude_min = latitude - (
            gps.latitude_deg * 100)
        gps.longitude_min = longitude - (
            gps.longitude_deg * 100)
        gps.bearing_deg = bearing
        gps.speed = speed
        lcm.publish('/gps', gps.encode())


if __name__ == "__main__":
    main()
