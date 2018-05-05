#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"
//#include "madgwick.hpp"

#define DT 1.0

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

    const float DECLINATION = -4.0f; // For Ann Arbor
    //Math::Vector3f V = { 27.451f, 14.048f, 8.852f };
    //Math::Vector3f V = { 29.448f, -1.122f, -23.356f };
    Math::Vector3f maxB = { 0 };
    Math::Vector3f minB = { 0 };
    while (true) {
        dbg.printf("Calibrate magnetometer...\r\n");
        dbg.printf("Roll the board in all orientations for 5 seconds.\r\n");
        wait(2);
        for (int i = 0; i < 5000; i++) {
            imu.read();
            Math::Vector3f m = imu.magnetometer();
            if (i == 0) {
                maxB.x = minB.x = m.x;
                maxB.y = minB.y = m.y;
                maxB.z = minB.z = m.z;
            }

            if (m.x > maxB.x) {
                maxB.x = m.x;
            }
            if (m.y > maxB.y) {
                maxB.y = m.y;
            }
            if (m.z > maxB.z) {
                maxB.z = m.z;
            }
            if (m.x < minB.x) {
                minB.x = m.x;
            }
            if (m.y < minB.y) {
                minB.y = m.y;
            }
            if (m.z < minB.z) {
                minB.z = m.z;
            }
            wait(0.001);
        }

        if (maxB.x > minB.x + 50 && maxB.y > minB.y + 50 && maxB.z > minB.z + 50) {
            dbg.printf("Calibration succeeded\r\n");
            dbg.printf("minB: (x=%.4f, y=%.4f, z=%.4f)\r\n", minB.x, minB.y, minB.z);
            dbg.printf("maxB: (x=%.4f, y=%.4f, z=%.4f)\r\n", maxB.x, maxB.y, maxB.z);
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
        dbg.printf("Magnetic field strength: %.4f uT\r\n", sqrtf(m.x*m.x + m.y*m.y + m.z*m.z));

        /*bool hasUpdated = false;
        if (m.x > maxB.x) {
            maxB.x = m.x;
            hasUpdated = true;
        }
        if (m.x < minB.x) {
            minB.x = m.x;
            hasUpdated = true;
        }
        if (m.y > maxB.y) {
            maxB.y = m.y;
            hasUpdated = true;
        }
        if (m.y < minB.y) {
            minB.y = m.y;
            hasUpdated = true;
        }
        if (m.z > maxB.z) {
            maxB.z = m.z;
            hasUpdated = true;
        }
        if (m.z < minB.z) {
            minB.z = m.z;
            hasUpdated = true;
        }

        if (hasUpdated) {
            dbg.printf("UPDATED CALIBRATION!\r\n");
        }*/

        Math::normalize_vec(a);
        float pitch = asin(-a.x);
        float roll = asin(a.y/cos(pitch));

        if (maxB.x - minB.x == 0 || maxB.y - minB.y == 0 || maxB.z - minB.z == 0) {
            dbg.printf("needs more calibration, rotate magnetometer more\r\n");
            continue;
        }

        float Bxc = (m.x - minB.x) / (maxB.x - minB.x) * 2 - 1;
        float Byc = (m.y - minB.y) / (maxB.y - minB.y) * 2 - 1;
        float Bzc = (m.z - minB.z) / (maxB.z - minB.z) * 2 - 1;

        dbg.printf("Calibrated magnetometer: (x=%.4f, y=%.4f, z=%.4f)\r\n", Bxc, Byc, Bzc);

        float Bx = Bxc*cos(pitch) + Bzc*sin(pitch);
        float By = Bxc*sin(roll)*sin(pitch) + Byc*cos(roll) - Bzc*sin(roll)*cos(pitch);

        dbg.printf("Projected: (x=%.4f, y=%.4f)\r\n", Bx, By);

        float yaw = 90.0f - atan2(Bx, By) * (180.0f/M_PI);
        yaw += DECLINATION;

        if (yaw < 0) {
            yaw += 360.0f;
        }

        dbg.printf("\r\nOrientation: (roll=%.4f, pitch=%.4f)\r\n", roll*(180.0f/M_PI), pitch*(180.0f/M_PI));
        dbg.printf("Yaw: %.4f\r\n", yaw);
        dbg.printf("\r\n");

        wait(DT);
    }
}
