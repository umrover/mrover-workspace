#include <mbed.h>
#include "can.hpp"
#include <cstdlib>

// temperature works
// pH works

/*
Thread temp_thread;
Thread pH_thread;
Thread conductivity_thread;
Thread O2_thread;
Thread CO2_thread;
Thread moisture_thread;

Serial temp_probe(PA_0, PA_1, 9600);
Serial pH_probe(PB_6, PB_7, 9600);
Serial conductivity_probe( // pin1, pin2, 9600);
*/

// Serial temp_probe(PA_0, PA_1, 9600);
// Serial pH_probe(PA_0, PA_1, 9600);
Serial conductivity_probe(PA_0, PA_1, 9600);

struct Gas_probe {
    AnalogIn probe;
    float slope;
    float intercept;
};

struct Moisture_probe {
    AnalogIn probe;
    float dry;
    float wet;
};

float O2_slope = 6.5625;
float O2_intercept = 0;

float CO2_low_slope = 2500;    // for low range (0-10,000ppm) mode
float CO2_high_slope = 25000;  // for high range (0-100,000ppm) mode
float CO2_intercept = 0;

float dry_val = 6.9;
float wet_val = 420;

Serial dbg(USBTX, USBRX, 9600);

// Function for reading Atlas sensors using Serial: pH, temperature, conductivity
void read_Serial_sensor(Serial &probe) {

    dbg.printf("Starting...\r\n");
	
    char sensorstring[30];    //a string to hold the data from the temp probe
    int i = 0;                // counter
    float data;
    while(true) {
        char inchar = probe.getc();                   // read a char from the sensor
        if(inchar != '\r') {					
            sensorstring[i] = inchar;                 // append char to the string
            i++;                                      // increment counter
        }
        else {                                        // when no more data to read
            data = atof(sensorstring);                // convert to float
            dbg.printf("%.3f\r\n", data);             // print value to Serial
            i = 0;                                    // reset counter
        }
    }
}

// Function for reading Vernier gas probes: O2, CO2
void read_gas_sensor(Gas_probe *sensor) {
    
    while(true) {
        float raw = (sensor->probe).read();                                    // read raw count
        // dbg.printf("%.3f\r\n", raw);
        float voltage = raw * 5;                                            // convert to voltage
        float sensor_value = (sensor->slope)*voltage + (sensor->intercept); // linear calibration

        dbg.printf("%.3f\r\n", sensor_value);                               // print value to Serial
        wait(1);
    }
}

// Function for reading moisture probe
void read_moisture_probe(Moisture_probe *sensor) {
    printf("%.3f\r\n", sensor->dry);
    printf("%.3f\r\n", sensor->wet);
    while(true) {
        float raw = (sensor->probe).read();
        dbg.printf("%.3f\r\n", raw);
        wait(0.1);
    }
}

int main() {
/*
    Gas_probe O2_probe;
    O2_probe.probe = AnalogIn( // pin );
    O2_probe.slope = O2_slope;
    O2_probe.intercept = O2_intercept;

    Gas_probe CO2_probe;
    CO2_probe.probe = AnalogIn( // pin);
    CO2_probe.slope = CO2_low_slope;
    CO2_probe.intercept = CO2_intercept;

    Moisture_probe moisture_probe;
    moisture_probe.probe = AnalogIn( // pin);
    moisture_probe.dry = dry_val;
    moisture_probe.wet = wet_val;

    temp_thread.start(callback(read_Serial_sensor, temp_probe));
    pH_thread.start(callback(read_Serial_sensor, pH_probe));
    conductivity_thread.start(callback(read_Serial_sensor, conductivity_probe));
    O2_thread.start(callback(read_gas_sensor, &O2_probe));
    CO2_thread.start(callback(read_gas_sensor, &CO2_probe));
    moisture_thread.start(callback(read_moisture_probe, &moisture_probe));
*/
    //Gas_probe O2_probe = {AnalogIn(PA_2), O2_slope, O2_intercept};
    Gas_probe CO2_probe = {AnalogIn(PA_2), CO2_low_slope, CO2_intercept};
    
    read_gas_sensor(&CO2_probe);

    while(true) {}
}
