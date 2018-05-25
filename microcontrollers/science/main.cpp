#include <mbed.h>
#include "frame_serial.hpp"
#include "pins.hpp"

Thread thread1, thread2;
Mutex mutex;

DigitalOut led(LED1_PROD);
AnalogIn O2_probe(ANALOGIN1_PROD);
AnalogIn CO2_probe(ANALOGIN2_PROD);
AnalogIn moisture_probe(ANALOGIN3_PROD);
Serial pH_probe(UART_TX2_PROD, UART_RX2_PROD);
Serial temperature_probe(UART_TX1_PROD, UART_RX1_PROD);
Serial conductivity_probe(UART_TX3_PROD, UART_RX3_PROD);
Serial serial(UART_TX5_PROD, UART_RX5_PROD, 115200);

char temp_buffer[256];
int temp_write_index = 0;
bool temp_first = true;

char cond_buffer[256];
int cond_write_index = 0;
bool cond_first = true;

char pH_buffer[256];
int pH_write_index = 0;
bool pH_first = true;

struct __attribute__((__packed__)) {
    float O2;
    float CO2;
    float moisture;
    float pH;
    float temperature;
    float conductivity;
} g_message;

void temperature_callback() {
    char c = temperature_probe.getc();
    if (c != '\r' && !temp_first) {
        temp_buffer[temp_write_index++] = c;
    } else {
        mutex.lock();
        g_message.temperature = atof(temp_buffer);
        mutex.unlock();
        temp_write_index = 0;
        temp_first = false;
    }
}

void pH_callback() {
    char c = pH_probe.getc();
    if (c != '\r' && !pH_first) {
        pH_buffer[pH_write_index++] = c;
    } else {
        mutex.lock();
        g_message.pH = atof(pH_buffer);
        mutex.unlock();
        pH_write_index = 0;
        pH_first = false;
    }
}

void cond_callback() {
    char c = conductivity_probe.getc();
    if (c != '\r' && !cond_first) {
        cond_buffer[cond_write_index++] = c;
    } else {
        mutex.lock();
        g_message.conductivity = atof(cond_buffer);
        mutex.unlock();
        cond_write_index = 0;
        cond_first = false;
    }
}

void get_gas() {
    float O2_slope = 44200.0f;
    float O2_intercept = -6630.0f;
    // float CO2_slope = 25000; // HIGH
    float CO2_slope = 2500.0f;  // LOW
    float CO2_intercept = 0;

    while(true) {
        float O2_voltage = O2_probe.read() * 5;
        float CO2_voltage = CO2_probe.read() * 5;

        mutex.lock();
        g_message.O2 = O2_voltage;
        g_message.CO2 = CO2_voltage;
        /*g_message.O2 = O2_slope * O2_voltage + O2_intercept;
        g_message.CO2 = CO2_slope * CO2_voltage + CO2_intercept;*/
        mutex.unlock();
        wait(0.5);
    }
}

void get_moisture() {
    // float wet = 0.41;
    // float dry = 0.83;
    while(true) {
        float raw = moisture_probe.read();
        mutex.lock();
        g_message.moisture = raw;
        mutex.unlock();
        wait(0.5);
    }
}

int main() {
    g_message.conductivity = -1023.f;
    g_message.temperature = -1023.f;
    g_message.pH = -1023.f;
    g_message.moisture = -1023.f;
    g_message.O2 = -1023.f;
    g_message.CO2 = -1023.f;
    for (int i = 0; i < 10; i++) {
        led = !led;
        wait(0.1);
    }

    thread1.start(get_gas);
    thread2.start(get_moisture);

    pH_probe.attach(&pH_callback);
    temperature_probe.attach(&temperature_callback);
    conductivity_probe.attach(&cond_callback);

    while(true) {
        mutex.lock();
        write_frame(serial, (uint8_t *) &g_message, sizeof(g_message));
        mutex.unlock();
        led = !led;
        wait(1);
    }
}
