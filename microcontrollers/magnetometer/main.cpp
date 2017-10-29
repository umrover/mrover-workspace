#include "mbed.h"

// Access pins on the board by p1, p2... or the onboard LEDs as LED!, LED2...
DigitalIn mag_pin1(p1);
DigitalOut led(LED1);

int main() {
    while (1) {
        led = mag_pin1.read();
        wait(0.5);
    }
}
