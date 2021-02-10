# Simple demo of the FXAS21002C gyroscope.
# Will print the gyroscope values every second.
import time
import board
import busio
import adafruit_fxas21002c
import adafruit_fxos8700
import lcm
import numpy
from rover_msgs import IMUData
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


def main():
    global lcm_
    lcm_ = lcm.LCM()
    imudata = IMUData()
    while True:
        print("start while")
        # Read gyroscope.
        gyro_x, gyro_y, gyro_z = sensor.gyroscope
        # Print values.
        print(
            "Gyroscope (radians/s): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
                gyro_x, gyro_y, gyro_z
            )
        )
        imudata.gyro_x_dps = gyro_x
        imudata.gyro_y_dps = gyro_y
        imudata.gyro_z_dps = gyro_z

        # Read acceleration & magnetometer.
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

        imudata.accel_x_g = accel_x
        imudata.accel_y_g = accel_y
        imudata.accel_z_g = accel_z

        imudata.mag_x_uT = mag_x
        imudata.mag_y_uT = mag_y
        imudata.mag_z_uT = mag_z

        # Bearing calculation
        bearing = -(numpy.arctan2(imudata.mag_y_uT, imudata.mag_x_uT) * (180.0/numpy.pi))
        if (bearing < 0):
            bearing += 360

        imudata.bearing_deg = bearing
        print("bearing: ", imudata.bearing_deg)

        # Delay for a second.
        time.sleep(1.0)

        lcm_.publish("/imu_data", imudata.encode())


if(__name__ == "__main__"):
    main()
