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
} g_message;

namespace ReadState {
    enum ReadState {
        DataType,
        Time,
        Status,
        LatDeg,
        LatMin,
        LatDir,
        LonDeg,
        LonMin,
        LonDir,
        SpeedKnots,
        TrackAngle,
        Waiting,
        Invalid
    };
}

class RTK {
public:

    // RTK() {
    //     memset(data_type, 0, sizeof(data_type));
    // }

    void read_char(char c) {
        next_state = state;

        //dbg.printf("here %d\r\n", count);
        ++count;

        if(state == ReadState::Invalid) wait_until_valid(c);

        if(state == ReadState::DataType) getDataType(c);

        if(state == ReadState::Time) getTime(c);

        if(state == ReadState::Status) getStatus(c);

        if(state == ReadState::LatDeg) getLatDeg(c);

        if(state == ReadState::LatMin) getLatMin(c);

        if(state == ReadState::LatDir) getLatDir(c);

        if(state == ReadState::LonDeg) getLonDeg(c);

        if(state == ReadState::LonMin) getLonMin(c);

        if(state == ReadState::LonDir) getLonDir(c);

        if(state == ReadState::SpeedKnots) getSpeedKnots(c);

        if(state == ReadState::TrackAngle) getTrackAngle(c);

        if(state == ReadState::Waiting) wait_until_end(c);

        state = next_state;
    }

private:
    bool valid = false;
    char data_type[7] = { 0 };
    size_t data_type_idx = 0;
    int latDegMultiplier, lonDegMultiplier;
    double latMinMultiplier, lonMinMultiplier, speedMultiplier, angleMultiplier;
    float LatDeg, LonDeg;
    double LatMin, LonMin, SpeedKnots, TrackAngle;
    int count = 0;

    ReadState::ReadState state = ReadState::Invalid;
    ReadState::ReadState next_state;

    void wait_until_valid(char c){
        if(c == '$') {
            data_type_idx = 0;
            next_state = ReadState::DataType;
        }
        return;
    }

    void getDataType(char c){
        if (c != ',') {
            data_type[data_type_idx] = c;
            data_type_idx++;
        } else {
            if(data_type[2] == 'R' && data_type[3] == 'M' && data_type[4] == 'C'){
                valid = true;
                next_state = ReadState::Time;
            } else {
                next_state = ReadState::Invalid;
            }
        }
        return;
    }

    void getTime(char c){
        if (c == ',') next_state = ReadState::Status;
        return;
    }

    void getStatus(char c){
        if(c == ',') {
            next_state = ReadState::LatDeg;
            latDegMultiplier = 10;
            LatDeg = 0;
        }
        return;
    }

    void getLatDeg(char c){
        LatDeg += (c - '0') * latDegMultiplier;
        if (latDegMultiplier <= 1) {
            next_state = ReadState::LatMin;
            latMinMultiplier = 10;
            LatMin = 0;
        }	
        latDegMultiplier /= 10;
        return;
    }

    void getLatMin(char c){
        if(c == ',') next_state = ReadState::LatDir;
        else if(c != '.'){
            LatMin += (c - '0') * latMinMultiplier;
            latMinMultiplier /= 10;
        }
        return;
    }

    void getLatDir(char c){
        if(c == 'S') LatDeg *= -1;
        else if(c == ',') {
            next_state = ReadState::LonDeg;
            lonDegMultiplier = 100;
            LonDeg = 0;
        }
        return;
    }

    void getLonDeg(char c){
        LonDeg += (c - '0') * lonDegMultiplier;
        if (lonDegMultiplier <= 1) {
            next_state = ReadState::LonMin;
            lonMinMultiplier = 10;
            LonMin = 0;
        }	
        lonDegMultiplier /= 10;
        return;
    }

    void getLonMin(char c){
        if(c == ',') next_state = ReadState::LonDir;
        else if(c != '.'){
            LonMin += (c - '0') * lonMinMultiplier;
            lonMinMultiplier /= 10;
        }
        return;
    }

    void getLonDir(char c){
        if(c == 'W') LonDeg *= -1;
        else if( c == ','){
            next_state = ReadState::SpeedKnots;
            speedMultiplier = 100;
            SpeedKnots = 0;
        }
        return;
    }

    void getSpeedKnots(char c){
        if(c == ',') {
            next_state = ReadState::TrackAngle;
            angleMultiplier = 100;
            TrackAngle = 0;
        }
        else if(c != '.'){
            SpeedKnots += (c - '0') * speedMultiplier;
            speedMultiplier /= 10;
        }
        return;
    }

    void getTrackAngle(char c){
        if(c == ',') next_state = ReadState::Waiting;
        else if(c != '.'){
            TrackAngle += (c - '0') * angleMultiplier;
            angleMultiplier /= 10;
        }
        return;
    }

    void wait_until_end(char c){
        if(c == '\n'){
            next_state = ReadState::Invalid;
            // print();
            update_globals();
            valid = false;
        }
        return;
    }

    // void print(){
    //     // dbg.printf("%d deg %2.7f min by %d deg %2.7f min moving at %2.4f knots along course %3.2f\n",
    //     //         LatDeg, LatMin, LonDeg, LonMin, SpeedKnots, TrackAngle);
    // }

    void update_globals() {
        mutex.lock();
        g_message.lat_deg = LatDeg;
        g_message.lat_min = LatMin;
        g_message.lon_deg = LonDeg;
        g_message.lon_min = LonMin;
        mutex.unlock();
    }
};


void get_IMU() {
    while(true) {
        Math::Vector3f V = { 0 };
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

            float pitch = atan2(-a.x, Az);
            float Bz = m.x*cosf(pitch) + m.z*sinf(pitch);

            float yaw = atan2(-By, Bz);
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
            g_message.bearing = bearing;
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
        /*uint8_t msg[28] = {0};
        memcpy(&msg[0], ((uint8_t *) &roll), sizeof(float));
        memcpy(&msg[4], ((uint8_t *) &pitch), sizeof(float));
        memcpy(&msg[8], ((uint8_t *) &bearing), sizeof(float));
        memcpy(&msg[12], ((uint8_t *) &LatDeg), sizeof(float));
        memcpy(&msg[16], ((uint8_t *) &LatMin), sizeof(float));
        memcpy(&msg[20], ((uint8_t *) &LonDeg), sizeof(float));
        memcpy(&msg[24], ((uint8_t *) &LonMin), sizeof(float));
        write_frame(serial, msg, 28);*/
        write_frame(serial, (uint8_t *) &g_message, sizeof(g_message));
        g_message.gps_read = false;
        mutex.unlock();
        wait(0.1);
    }
}
