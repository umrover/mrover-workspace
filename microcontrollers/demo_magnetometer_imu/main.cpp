#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"
//#include "madgwick.hpp"

#define DT 2.0

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);
Serial dbg(USBTX, USBRX, 115200);
Imu imu(PB_7, PB_6);

int main() {
    red_led = false;
    green_led = true;
    while (!imu.valid()) {
        green_led = false;
        red_led = true;
        dbg.printf("invalid IMU, check connection\r\n");
        imu.init();
        dbg.printf("reconnecting...\r\n");
        wait(1.0);
    }

    Math::Vector3f V = { 0 };
    Math::Vector3f maxV = { 0 };
    Math::Vector3f minV = { 0 };
    while (true) {
        dbg.printf("Calibrate magnetometer...\r\n");
        dbg.printf("Roll the board in all orientations for 5 seconds.\r\n");
        wait(2);
        for (int i = 0; i < 5000; i++) {
            imu.read();
            Math::Vector3f m = imu.magnetometer();
            if (i == 0) {
                maxV.x = minV.x = m.x;
                maxV.y = minV.y = m.y;
                maxV.z = minV.z = m.z;
            }

            if (m.x > maxV.x) {
                maxV.x = m.x;
            }
            if (m.y > maxV.y) {
                maxV.y = m.y;
            }
            if (m.z > maxV.z) {
                maxV.z = m.z;
            }
            if (m.x < minV.x) {
                minV.x = m.x;
            }
            if (m.y < minV.y) {
                minV.y = m.y;
            }
            if (m.z < minV.z) {
                minV.z = m.z;
            }
            wait(0.001);
        }

        if (maxV.x > minV.x + 50 && maxV.y > minV.y + 50 && maxV.z > minV.z + 50) {
            V.x = (maxV.x + minV.x)/2.0f;
            V.y = (maxV.y + minV.y)/2.0f;
            V.z = (maxV.z + minV.z)/2.0f;
            dbg.printf("Calibration succeeded\r\n");
            break;
        } else {
            dbg.printf("Calibration failed\r\n");
            wait(2);
        }
    }

    while (true) {
        imu.read();
        Math::Vector3f a = imu.accelerometer();
        Math::Vector3f m = imu.magnetometer();
        Math::Vector3f g = imu.gyroscope();
        dbg.printf("-----------RAW VALUES------------\r\n");
        dbg.printf("A: (x=%.4f, y=%.4f, z=%.4f) m/s^2\r\n", a.x, a.y, a.z);
        dbg.printf("M: (x=%.4f, y=%.4f, z=%.4f) uT\r\n", m.x, m.y, m.z);
        //dbg.printf("G: (x=%.4f, y=%.4f, z=%.4f) deg/s\r\n", g.x, g.y, g.z);
        dbg.printf("\r\n");

        /*float roll = atan2f(a.y, a.z);
        float pitch = atan2f(-a.x, a.y*sinf(roll) + a.z*cosf(roll));

        float yaw = atan2f(
                (m.z - V.z)*sinf(roll) - (m.y - V.y)*cosf(roll),
                (m.x - V.x)*cosf(pitch) + (m.y - V.y)*sinf(pitch)*sinf(roll) + (m.z - V.z)*sinf(pitch)*cosf(roll));
        float bearing = yaw * (180.0f/M_PI);*/

        m.x -= V.x;
        m.y -= V.y;
        m.z -= V.z;

        float roll = atan2(a.y, a.z);
        float By = m.y*cosf(roll) - m.z*sinf(roll);
        float Mz = m.z*cosf(roll) + m.y*sinf(roll);
        float Az = a.y*sinf(roll) + a.z*cosf(roll);

        float pitch = atan2(-a.x, Az);
        float Bz = m.x*cosf(pitch) + m.z*sinf(pitch);

        float yaw = atan2(-By, Bz);
        float bearing = yaw * (180.0f/M_PI);
        if (bearing < 0) {
            bearing += 360.0f;
        }

        dbg.printf("Orientation: (roll=%.4f, pitch=%.4f)\r\n", roll*(180.0f/M_PI), pitch*(180.0f/M_PI));
        dbg.printf("Yaw (bearing): %.4f\r\n", bearing);
        dbg.printf("\r\n");

        wait(DT);
    }
}
