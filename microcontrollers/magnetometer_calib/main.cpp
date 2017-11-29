#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"

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

    dbg.printf("Got valid IMU\r\n");

    Math::Vector3f V = { 0 };
    Math::Vector3f maxV = { 0 };
    Math::Vector3f minV = { 0 };

    while (true) {
        dbg.printf("Calibrate magnetometer...\r\n");
        dbg.printf("Roll the board in all orientations for 5 seconds.\r\n");
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
            dbg.printf("Constants:\r\n");
            dbg.printf("\tV.x = %.8f\r\n", V.x);
            dbg.printf("\tV.y = %.8f\r\n", V.y);
            dbg.printf("\tV.z = %.8f\r\n", V.z);
        } else {
            dbg.printf("Calibration failed\r\n");
        }
        dbg.printf("\r\n\r\nCalibration will restart in another 10 seconds\r\n");
        wait(10);
    }
}
