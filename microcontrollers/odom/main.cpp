#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"
#include "nmea.hpp"
#include "pins.hpp"
#include "frame_serial.hpp"

DigitalOut led(LED1_PROD);
// Serial dbg(USBTX, USBRX, 9600);
Serial serial(UART_TX1_PROD, UART_RX1_PROD, 115200);
Serial gps_in(UART_TX2_PROD, UART_RX2_PROD, 115200);
Imu imu(I2C_SDA_PROD, I2C_SCL_PROD);
Thread thread1;
Mutex mutex;
RMCParser parser;

struct __attribute__((__packed__)) {
    float roll;
    float pitch;
    float bearing;
    int lat_deg;
    float lat_min;
    int lon_deg;
    float lon_min;
    bool gps_read;
    bool imu_read;
} g_message;

void get_IMU() {
    const float DECLINATION = -4.0f;
    while(true) {
        //Math::Vector3f V = { 0 };
        Math::Vector3f maxB = { 62.9, 66.7, 428.3 };
        Math::Vector3f minB = { -34.3, -31.3, 336.4 };
        while (!imu.valid()) {
            led = false;
            // dbg.printf("invalid IMU, check connection\r\n");
            imu.init();
            // dbg.printf("reconnecting...\r\n");
            wait(1.0);
        }

        while(true) {
            if (!imu.valid()) {
                break;
            }
            imu.read();
            Math::Vector3f a = imu.accelerometer();
            Math::Vector3f m = imu.magnetometer();

            Math::normalize_vec(a);
            float pitch = asin(-a.x);
            float roll = asin(a.y/cos(pitch));

            float Bxc = (m.x - minB.x) / (maxB.x - minB.x) * 2 - 1;
            float Byc = (m.y - minB.y) / (maxB.y - minB.y) * 2 - 1;
            float Bzc = (m.z - minB.z) / (maxB.z - minB.z) * 2 - 1;

            float Bx = Bxc*cos(pitch) + Bzc*sin(pitch);
            float By = Bxc*sin(roll)*sin(pitch) + Byc*cos(roll) - Bzc*sin(roll)*cos(pitch);

            float yaw = 90.0f - atan2(Bx, By) * (180.0f/M_PI);
            yaw += DECLINATION;

            if (yaw < 0) {
                yaw += 360.0f;
            }
            
            mutex.lock();
            g_message.roll = roll;
            g_message.pitch = pitch;
            g_message.bearing = yaw;
            g_message.imu_read = true;
            mutex.unlock();
        }
    }
}

void gps_callback() {
    char c = gps_in.getc();
    if (parser.feed(c)) {
        mutex.lock();
        g_message.gps_read = true;
        g_message.lat_deg = parser.latitude_deg();
        g_message.lat_min = parser.latitude_min();
        g_message.lon_deg = parser.longitude_deg();
        g_message.lon_min = parser.longitude_min();
        mutex.unlock();
    }
}

int main() {

    thread1.start(get_IMU);
    
    gps_in.attach(&gps_callback);

    while(true) {
        mutex.lock();
        write_frame(serial, (uint8_t *) &g_message, sizeof(g_message));
        g_message.gps_read = false;
        g_message.imu_read = false;
        mutex.unlock();
        wait(0.5);
    }
}
