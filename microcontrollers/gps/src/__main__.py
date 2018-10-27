import serial
from rover_common import aiolcm
from rover_msgs import Odometry


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

        odometry = Odometry()
        serialPort.read_until(b',')[:-1]
        serialPort.read_until(b',')[:-1]

        hasFix = serialPort.read_until(b',')[:-1]
        if hasFix != b'A':
            continue

        latitude = float(serialPort.read_until(b',')[:-1])
        serialPort.read_until(b',')[:-1]
        longitude = float(serialPort.read_until(b',')[:-1])
        serialPort.read_until(b',')[:-1]
        speed = float(serialPort.read_until(b',')[:-1])
        bearing = float(serialPort.read_until(b',')[:-1])

        odometry.latitude_deg = int(latitude/100)
        odometry.longitude_deg = int(longitude/100)
        odometry.latitude_min = latitude - (
            odometry.latitude_deg * 100)
        odometry.longitude_min = longitude - (
            odometry.longitude_deg * 100)
        odometry.bearing_deg = bearing
        odometry.speed = speed
        lcm.publish('/odometry', odometry.encode())


if __name__ == "__main__":
    main()
