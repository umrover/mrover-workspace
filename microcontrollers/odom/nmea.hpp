#pragma once

#ifndef MBED_BUILD_TIMESTAMP
#include <cstdlib>
using std::size_t;
#endif

class RMCParser {
    public:
        RMCParser();
        bool feed(char c);

        int latitude_deg() const { return this->lat_deg_; }
        float latitude_min() const { return this->lat_min_; }
        
        int longitude_deg() const { return this->lon_deg_; }
        float longitude_min() const { return this->lon_min_; }

        enum State {
            WaitForHeader,
            ReadingHeader,
            FixTime,
            ReceiverWarning,
            LatDeg,
            LatMin,
            LatDir,
            LonDeg,
            LonMin,
            LonDir
        };

        State state_;
    private:
        static const size_t HEADER_TYPE_LEN = 7;

        char header_type_[HEADER_TYPE_LEN];
        size_t header_type_offset_;
        int lat_deg_;
        float lat_min_;
        int lon_deg_;
        float lon_min_;

        int lat_deg_mult_;
        float lat_min_mult_;
        int lon_deg_mult_;
        float lon_min_mult_;

        State add_to_deg(char c, int * deg_val, int * mult, float * min_mult, State next);
        State add_to_min(char c, float * min_val, float * mult, State next);
        State add_dir(char c, int * deg_val, float * min_val, State next);
};


class GSVParser {
    public:
        GSVParser();
        bool feed(char c);

        int num_satellites() { return this->num_sats_; }

        enum State {
            WaitForHeader,
            ReadingHeader,
            NumMessages,
            MessageIndex,
            NumSatellites
        };

        State state_;
    private:
        static const size_t HEADER_TYPE_LEN = 7;
        char header_type_[HEADER_TYPE_LEN];
        size_t header_type_offset_;

        int num_sats_;

        int num_sats_mult_;
};
