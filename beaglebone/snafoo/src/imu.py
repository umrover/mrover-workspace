import board
import busio
import adafruit_fxas21002c
import adafruit_fxos8700
import numpy

XACCEL = 0
YACCEL = 1
ZACCEL = 2

XGYRO = 3
YGYRO = 4
ZGYRO = 5

XMAG = 6
YMAG = 7
ZMAG = 8

# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
print("sees board")
sensor = adafruit_fxas21002c.FXAS21002C(i2c)
print("sees gyro")
sensor2 = adafruit_fxos8700.FXOS8700(i2c)
print("sees a/m")
# Optionally create the sensor with a different gyroscope range (the
# default is 250 DPS, but you can use 500, 1000, or 2000 DPS values):
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_500DPS)
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_1000DPS)
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)
# Main loop will read the gyroscope values every second and print them out.

def get_data():
    # Read gyroscope.
    gyro_x, gyro_y, gyro_z = sensor.gyroscope
    # Print values.
    print(
        "Gyroscope (radians/s): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
            gyro_x, gyro_y, gyro_z
        )
    )

    accel_x, accel_y, accel_z = sensor2.accelerometer
    mag_x, mag_y, mag_z = sensor2.magnetometer
    # Print values.
    print(
        "Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
            accel_x, accel_y, accel_z
        )
    )

    print(
        "Magnetometer (uTesla): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
            mag_x, mag_y, mag_z
        )
    )    

    return numpy.array([accel_x, accel_y, accel_z, gyro_x,
                    gyro_y, gyro_z, mag_x, mag_y, mag_z])
