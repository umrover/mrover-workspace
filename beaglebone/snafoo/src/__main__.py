# Simple demo of the FXAS21002C gyroscope.
# Will print the gyroscope values every second.
import time
import board
import busio
import adafruit_fxas21002c
# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_fxas21002c.FXAS21002C(i2c)
# Optionally create the sensor with a different gyroscope range (the
# default is 250 DPS, but you can use 500, 1000, or 2000 DPS values):
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_500DPS)
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_1000DPS)
# sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)
# Main loop will read the gyroscope values every second and print them out.
while True:
    # Read gyroscope.
    gyro_x, gyro_y, gyro_z = sensor.gyroscope
    # Print values.
    print(
        "Gyroscope (radians/s): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
            gyro_x, gyro_y, gyro_z
        )
    )

    # Read acceleration & magnetometer.
    accel_x, accel_y, accel_z = sensor.accelerometer
    mag_x, mag_y, mag_z = sensor.magnetometer
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

    # Delay for a second.
    time.sleep(1.0)
