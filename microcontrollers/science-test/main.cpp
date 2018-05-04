#include <mbed.h>
#include "pins.hpp"

Thread thread1, thread2, thread3, thread4, thread5;
Mutex mutex;

DigitalOut led(LED1_DEV);
AnalogIn O2_probe(ANALOGIN1_DEV);
AnalogIn CO2_probe(ANALOGIN2_DEV);
// AnalogIn moisture_probe(ANALOGIN2_DEV);
// Serial pH_probe(UART_TX2_PROD, UART_RX2_PROD);
// Serial temperature_probe(UART_TX2_PROD, UART_RX2_PROD);
// Serial conductivity_probe(UART_TX2_PROD, UART_RX2_PROD);
// Serial serial(UART_TX1_PROD, UART_RX1_PROD, 115200);
Serial dbg(USBTX, USBRX, 115200);

struct __attribute__((__packed__)) {
    float O2;
    float CO2;
    float moisture;
    float pH;
    float temperature;
    float conductivity;
} g_message;

// void get_temperature() {
//     char buffer[256];
//     int i = 0;
//     while (true) {
//         char inchar = temperature_probe.getc();
//         if (inchar != '\r') {                    
//             buffer[i] = inchar;
//             i++;
//         } else {
//             mutex.lock();
//             g_message.temperature = atof(buffer);
//             mutex.unlock();
//             i = 0;
//             wait(0.1);
//         }
//     }
// }

// void get_pH() {
//     char buffer[256];
//     int i = 0;
//     while (true) {
//         char inchar = pH_probe.getc();
//         if (inchar != '\r') {
//             buffer[i] = inchar;
//             i++;
//         } else {
//             mutex.lock();
//             g_message.pH = atof(buffer);
//             mutex.unlock();
//             i = 0;
//             wait(0.5);
//         }
//     }
// }

// void get_conductivity() {
//     char buffer[30];
//     int i = 0;
//     while (true) {
//         char inchar = conductivity_probe.getc();
//         if (inchar != '\r') {
//             buffer[i] = inchar;
//             i++;
//         } else {
//             g_message.conductivity = atof(buffer);
//             i = 0;
//             wait(0.5);
//         }
//     }
// }

void get_gas() {
    float O2_slope = 221000.0f;
    float O2_intercept = -6630.0f;
    // float CO2_slope = 25000; // HIGH
    float CO2_slope = 2500.0f;  // LOW
    float CO2_intercept = 0;

    while(true) {
        float O2_voltage = O2_probe.read();
        g_message.O2 = O2_slope * O2_voltage + O2_intercept;

        
        float CO2_voltage = CO2_probe.read() * 5;
        g_message.CO2 = CO2_slope * CO2_voltage + CO2_intercept;

        dbg.printf("O2: %.2f\r\n", g_message.O2);
        // dbg.printf("CO2: %.2f\r\n", g_message.CO2);
        wait(1);
    }
}

// void get_moisture() {
//     float wet = 0.41;
//     float dry = 0.83;
//     while(true) {
//         float raw = moisture_probe.read();
//         g_message.moisture = raw;
//         dbg.printf("Moisture: %.4f\r\n", g_message.moisture);
//         wait(0.5);
//     }
// }

int main() {
    for (int i = 0; i < 10; i++) {
        led = !led;
        wait(0.1);
    }

    thread1.start(get_gas);
    // thread2.start(get_moisture);
    // thread3.start(get_pH);
    // thread4.start(get_temperature);
    // thread5.start(get_conductivity);

    while(true) {
        led = !led;
        wait(1);
    }
}