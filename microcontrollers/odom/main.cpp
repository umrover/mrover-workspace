#include <mbed.h>

DigitalOut green_led(LED1);
DigitalOut red_led(LED2);
Serial dbg(USBTX, USBRX, 9600);
Serial gps_in(PB_6, PB_7, 115200);

enum class ReadState {
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

class RTK {
public:

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

    ReadState state = ReadState::Invalid;
    ReadState next_state;

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
};

int main() {

    // put your setup code here, to run once:
    RTK rover;
    red_led = true;
    
    //char c = gps_in.getc();

    while(true) {
        // put your main code here, to run repeatedly:
        while (gps_in.readable()) {
            //dbg.printf("readable now\r\n");
            char c = gps_in.getc();
            rover.read_char(c);
        }
    }
}
