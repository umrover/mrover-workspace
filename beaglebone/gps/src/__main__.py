import serial
# import string
import lcm
from rover_msgs import GPSData
lcm_ = lcm.LCM()

baud = 38400


def main():

    gps = GPSData()
    with serial.Serial(port="/dev/ttyS4", bytesize=serial.EIGHTBITS,
                       stopbits=serial.STOPBITS_ONE,
                       parity=serial.PARITY_NONE, xonxoff=False, rtscts=False,
                       dsrdtr=False, baudrate=baud) as ser:
        while(True):

            # reads in data as a string
            data = str(ser.read_until())

            # removes extra quotes around the data string
            datastring = data[2:-1]

            # splits up correctly formatted string into a list of strings
            datalist = datastring.split(',')
            # print("datalist is of type: ", type(datalist), "size is: ",
            # len(datalist))

            if(datalist[0] == "$GNRMC"):
                #  print("Transmission Type:", datalist[0], "timeStamp:",
                # datalist[1],
                # "Status (A=active, V = void):", datalist[2],
                # "Latitude:", datalist[3], ",", datalist[4], "Longitude:",
                # datalist[5],
                # ",", datalist[6], "Ground speed(knots):", datalist[7],
                # "Track angle(in Degrees True):", datalist[8])
                gps.timeStamp = datalist[1]
                gps.latitude = datalist[3]
                gps.latitudeDirection = datalist[4]
                gps.longitude = datalist[5]
                gps.longitudeDirection = datalist[6]
                gps.trackAngle = datalist[8]

            elif(datalist[0] == "$GNVTG"):
                # print("Transmission Type:", datalist[0],
                # "True track made good:", datalist[1],
                # "Magnetic track:", datalist[2], "Ground speed(knots):",
                # datalist[3],
                # "Ground speed(km/h):", datalist[4])

                gps.groundSpeed = datalist[4]
            elif(datalist[0] == "$GNGGA"):
                # print("Transmission Type:", datalist[0],
                # "Fix Quality(4=RTK):", datalist[6],
                # "Number of satellites tracked:", datalist[7],
                # "Altitude (above mean sea level):", datalist[9],
                # "time since last DGPS update (s):", datalist[11])

                gps.quality = datalist[6]
                gps.altitude = datalist[9]
            elif(datalist[0] == "$GPGSV"):
                # print("Transmission Type:", datalist[0],
                # "# of sentences for full data:", datalist[1],
                # "Sentence _ of total:", datalist[2],
                # "# satellites in view:", datalist[3])
                gps.satellitesInView = datalist[3]

            lcm_.publish('/gps_data', gps.encode())


if (__name__ == "__main__"):
    main()
