#include <mbed.h>
#include "frame_serial.hpp"
#include "pins.hpp"

Thread thread1, thread2, thread3, thread4, thread5;

DigitalOut led(LED1_PROD);
AnalogIn O2_probe(PA_0);
AnalogIn CO2_probe(PA_1);
AnalogIn moisture_probe(PA_2);
Serial pH_probe(PB_6, PB_7);
Serial temperature_probe(PB_6, PB_7);
Serial conductivity_probe(PB_6, PB_7);
Serial serial(UART_TX1_PROD, UART_RX1_PROD, 115200);

float O2 = 0;
float CO2 = 0;
float moisture = 0;
float pH = 0;
float temperature = 0;
float conductivity = 0;

void get_temperature() {
    char buffer[30];
    int i = 0;
    while (true) {
        char inchar = temperature_probe.getc();
        if (inchar != '\r') {                    
            buffer[i] = inchar;
            i++;
        } else {
            temperature = atof(buffer);
            i = 0;
            wait(0.5);
        }
    }
}

void get_pH() {
    char buffer[30];
    int i = 0;
    while (true) {
        char inchar = pH_probe.getc();
        if (inchar != '\r') {                    
            buffer[i] = inchar;
            i++;
        } else {
            pH = atof(buffer);
            i = 0;
            wait(0.5);
        }
    }
}

void get_conductivity() {
    char buffer[30];
    int i = 0;
    while (true) {
        char inchar = conductivity_probe.getc();
        if (inchar != '\r') {                    
            buffer[i] = inchar;
            i++;
        } else {
            conductivity = atof(buffer);
            i = 0;
            wait(0.5);
        }
    }
}

void get_gas() {
    float O2_slope = 6.5625;
    float O2_intercept = 0;
    float CO2_slope = 2500;
    float CO2_intercept = 0;

    while(true) {
        float O2_voltage = O2_probe.read() * 5;
        O2 = O2_slope * O2_voltage + O2_intercept;
        
        float CO2_voltage = CO2_probe.read() * 5;
        CO2 = CO2_slope * CO2_voltage + CO2_intercept;
        wait(0.5);
    }
}

void get_moisture() {
    while(true) {
        float raw = moisture_probe.read();
        // TODO: figure out how to interpret moisture reading
        wait(0.5);
    }
}

void send_msg() {
    uint8_t msg[24] = {0};
    memcpy(&msg[0], ((uint8_t *) &O2), sizeof(float));
    memcpy(&msg[4], ((uint8_t *) &CO2), sizeof(float));
    memcpy(&msg[8], ((uint8_t *) &moisture), sizeof(float));
    memcpy(&msg[12], ((uint8_t *) &pH), sizeof(float));
    memcpy(&msg[16], ((uint8_t *) &temperature), sizeof(float));
    memcpy(&msg[20], ((uint8_t *) &conductivity), sizeof(float));
    write_frame(serial, msg, 24);
    led = !led;
    wait(1);
}

int main() {
    thread1.start(get_gas);
    thread2.start(get_moisture);
    thread3.start(get_pH);
    thread4.start(get_temperature);
    thread5.start(get_conductivity);

    while(true) {
        send_msg();
    }
}