#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"
#include "nmea.hpp"
#include "pins.hpp"
#include "frame_serial.hpp"

DigitalOut led(LED1_PROD);
// Serial dbg(USBTX, USBRX, 9600);
Serial serial(RTK_UART_TX1_PROD, RTK_UART_RX1_PROD, 115200);
Serial gps_in(RTK_UART_TX2_PROD, RTK_UART_RX2_PROD, 115200);
Imu imu(IMU_I2C_SDA_PROD, IMU_I2C_SCL_PROD);
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
    while(true) {
        // Math::Vector3f V = { 0 };
        Math::Vector3f V = { 30, 20, 110 };
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
            // Math::Vector3f g = imu.gyroscope();
            // dbg.printf("-----------RAW VALUES------------\r\n");
            // dbg.printf("A: (x=%.4f, y=%.4f, z=%.4f) m/s^2\r\n", a.x, a.y, a.z);
            // dbg.printf("M: (x=%.4f, y=%.4f, z=%.4f) uT\r\n", m.x, m.y, m.z);
            //dbg.printf("G: (x=%.4f, y=%.4f, z=%.4f) deg/s\r\n", g.x, g.y, g.z);
            // dbg.printf("\r\n");

            m.x -= V.x;
            m.y -= V.y;
            m.z -= V.z;

            float roll = atan2(a.y, a.z);
            float By = m.y*cosf(roll) - m.z*sinf(roll);
            float Mz = m.z*cosf(roll) + m.y*sinf(roll);
            float Az = a.y*sinf(roll) + a.z*cosf(roll);

            float pitch = atan(-a.x / Az);
            float Bx = m.x*cosf(pitch) + m.z*sinf(pitch);

            float yaw = atan2(-By, Bx);
            float bearing = yaw * (180.0f/M_PI);
            if (bearing < 0) {
                bearing += 360.0f;
            }

            roll = roll*(180.0f/M_PI);
            if (roll < 0) {
                roll += 360.0f;
            }

            pitch = pitch*(180.0f/M_PI);
            if (pitch < 0) {
                pitch += 360.0f;
            }
            mutex.lock();
            g_message.roll = roll;
            g_message.pitch = pitch;
            g_message.bearing = 360.0f - bearing; // empirically derived hack
            g_message.imu_read = true;
            mutex.unlock();

            // dbg.printf("Orientation: (roll=%.4f, pitch=%.4f)\r\n", roll*(180.0f/M_PI), pitch*(180.0f/M_PI));
            // dbg.printf("Yaw (bearing): %.4f\r\n", bearing);
            // dbg.printf("\r\n");
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
