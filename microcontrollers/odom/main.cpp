#include <mbed.h>
#include "math_3d.hpp"
#include "imu.hpp"

#define DT 2.0
#define LAT_MIN     0x50100000
#define LAT_DEG     0x50200000
#define LON_MIN     0x50300000
#define LON_DEG     0x50400000
#define SPEED_KNOTS 0x50500000
#define TRACK_ANGLE 0x50600000
#define ROLL        0x50700000
#define PITCH       0x50800000
#define YAW         0x50800000

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);
Serial dbg(USBTX, USBRX, 9600);
Serial gps_in(PA_0, PA_1, 115200);
Imu imu(PB_7, PB_6);
Thread thread;
CAN can1(PB_9, PB_10); // Change these pins

int rtk_counter = 0;
int imu_counter = 0;


float fxp_10_22_to_float(int fxp) {
    float CONVERSION_CONST = 0.0000002384185791015625;
    float raw = float(fxp);
    return raw * CONVERSION_CONST;
}


int float_to_fxp_10_22(float val) {
    float CONVERSION_CONST = float(0x400000);
    return int(val * CONVERSION_CONST);
}


void send_msg(int arbId, int counter, float data, int bytes_to_send=8) {
    // Encode msg into CAN data format
    int encoded_data = float_to_fxp_10_22(data);

    char msg[8];
    memcpy(&msg[0], &encoded_data, sizeof(int));
    memcpy(&msg[4], &counter, sizeof(int));

    // Send the message
    if (can1.write(CANMessage(arbId, msg, bytes_to_send))) {
        dbg.printf("Message sent: %d\n", data);
    }
}


void recv_msg() {
    CANMessage msg;
    if (can1.read(msg)) {
        int data;
        memcpy(&data, &msg.data, sizeof(int));
        dbg.printf("Message received: %d\n", fxp_10_22_to_float(data));
    }
}


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
    int LatDeg, LonDeg;
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
            print();
            send_can_msg();
            valid = false;
        }
        return;
    }

    void print(){
        red_led = false;
        green_led = true;
        dbg.printf("%d deg %2.7f min by %d deg %2.7f min moving at %2.4f knots along course %3.2f\n",
                LatDeg, LatMin, LonDeg, LonMin, SpeedKnots, TrackAngle);
    }

    void send_can_msg(){
        send_msg(LAT_DEG, rtk_counter, LatDeg);
        send_msg(LAT_MIN, rtk_counter, LatMin);
        send_msg(LON_DEG, rtk_counter, LonDeg);
        send_msg(LON_MIN, rtk_counter, LonMin);
        send_msg(SPEED_KNOTS, rtk_counter, SpeedKnots);
        send_msg(TRACK_ANGLE, rtk_counter, TrackAngle);

        ++rtk_counter;
    }
};


void get_IMU() {
    while(true) {
        Math::Vector3f V = { 0 };
        while (!imu.valid()) {
            green_led = false;
            red_led = true;
            dbg.printf("invalid IMU, check connection\r\n");
            imu.init();
            dbg.printf("reconnecting...\r\n");
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
            dbg.printf("-----------RAW VALUES------------\r\n");
            dbg.printf("A: (x=%.4f, y=%.4f, z=%.4f) m/s^2\r\n", a.x, a.y, a.z);
            dbg.printf("M: (x=%.4f, y=%.4f, z=%.4f) uT\r\n", m.x, m.y, m.z);
            //dbg.printf("G: (x=%.4f, y=%.4f, z=%.4f) deg/s\r\n", g.x, g.y, g.z);
            dbg.printf("\r\n");

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

            dbg.printf("Orientation: (roll=%.4f, pitch=%.4f)\r\n", roll*(180.0f/M_PI), pitch*(180.0f/M_PI));
            dbg.printf("Yaw (bearing): %.4f\r\n", bearing);
            dbg.printf("\r\n");

            send_msg(ROLL, imu_counter, roll*(180.0f/M_PI));
            send_msg(PITCH, imu_counter, pitch*(180.0f/M_PI));
            send_msg(YAW, imu_counter, bearing);

            ++imu_counter;

            // wait(DT);
        }
    }
}


int main() {

    // put your setup code here, to run once:
    RTK rtk;

    thread.start(get_IMU);

    while(true) {
        // put your main code here, to run repeatedly:
        while (gps_in.readable()) {
            // dbg.printf("readable now\r\n");
            char c = gps_in.getc();
            rtk.read_char(c);
            // wait(DT / 2);
        }
    }
}
