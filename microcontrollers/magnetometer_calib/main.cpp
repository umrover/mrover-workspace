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

    dbg.printf("Bx,By,Bz\r\n");

    wait(1.0);

    while (true) {
        imu.read();
        Math::Vector3f B = imu.magnetometer();

        dbg.printf("%.4f,%.4f,%.4f\r\n", B.x, B.y, B.z);
        wait(0.1);
    }
}
