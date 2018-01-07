#include "mbed.h"

// Access pins on the board by p1, p2... or the onboard LEDs as LED1, LED2...
DigitalIn mag_pin1(p1);
DigitalOut led(LED1);

int main() {
    // The magnetometer will communicate via I2C, this is only reading a single bit at a time
    // The purpose of this is to see if the input from the magnetometer can change the LED at all
    while (1) {
        led = mag_pin1.read();
        wait(0.5);
    }
}
