#ifdef MBED_BUILD_TIMESTAMP
#include "mbed.h"
#else
#include <cstring>
#include <cstdlib>
using std::memset;
using std::size_t;
#endif
#include "nmea.hpp"

static bool digit(char c) {
    return c >= '0' && c <= '9';
}

static bool alpha(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

RMCParser::RMCParser() : 
    state_(WaitForHeader),
    header_type_offset_(0),
    lat_deg_(0),
    lat_min_(0),
    lon_deg_(0),
    lon_min_(0)
{
}

bool RMCParser::feed(char c) {
    State next_state = this->state_;
    bool completed = false;
    switch (this->state_) {
        case WaitForHeader:
            if (c == '$') {
                this->header_type_offset_ = 0;
                memset(this->header_type_, 0, sizeof(this->header_type_));
                this->lat_deg_ = 0;
                this->lat_min_ = 0;
                this->lon_deg_ = 0;
                this->lon_min_ = 0;
                next_state = ReadingHeader;
            }
            break;
        case ReadingHeader:
            if (c == ',' || this->header_type_offset_ >= 5) {
                if (this->header_type_offset_ < 5) {
                    // Invalid, wait for another message.
                    next_state = WaitForHeader;
                } else if (this->header_type_[2] == 'R' &&
                        this->header_type_[3] == 'M' &&
                        this->header_type_[4] == 'C') {
                    next_state = FixTime;
                } else {
                    next_state = WaitForHeader;
                }
            } else if (alpha(c)) {
                if (this->header_type_offset_ >= HEADER_TYPE_LEN) {
                    // Invalid
                    next_state = WaitForHeader;
                } else {
                    this->header_type_[this->header_type_offset_++] = c;
                    // Stay in current state
                }
            } else {
                // Invalid, wait for another message.
                next_state = WaitForHeader;
            }
            break;
        case FixTime:
            if (c == ',') {
                next_state = ReceiverWarning;
            } else {
                if (!digit(c) && c != '.') {
                    // Invalid
                    next_state = WaitForHeader;
                }
            }
            break;
        case ReceiverWarning:
            if (c == ',') {
                this->lat_deg_mult_ = 10;
                next_state = LatDeg;
            } else if (c != 'A' && c != 'V') {
                // Invalid
                next_state = WaitForHeader;
            }
            break;
        case LatDeg:
            next_state = add_to_deg(c, &this->lat_deg_, &this->lat_deg_mult_, &this->lat_min_mult_, LatMin);
            break;
        case LatMin:
            next_state = add_to_min(c, &this->lat_min_, &this->lat_min_mult_, LatDir);
            break;
        case LatDir:
            next_state = add_dir(c, &this->lat_deg_, &this->lat_min_, LonDeg);
            break;
        case LonDeg:
            next_state = add_to_deg(c, &this->lon_deg_, &this->lon_deg_mult_, &this->lon_min_mult_, LonMin);
            break;
        case LonMin:
            next_state = add_to_min(c, &this->lon_min_, &this->lon_min_mult_, LonDir);
            break;
        case LonDir:
            next_state = add_dir(c, &this->lon_deg_, &this->lon_min_, WaitForHeader);
            if (next_state == WaitForHeader) {
                completed = true;
            }
            break;
    }
    this->state_ = next_state;
    return completed;
}

RMCParser::State RMCParser::add_to_deg(char c, int * deg_val, int * mult, float * min_mult, RMCParser::State next) {
    if (!digit(c)) {
        // Invalid
        return WaitForHeader;
    }
    *deg_val += (c - '0') * (*mult);
    if ((*mult) <= 1) {
        *min_mult = 10;
        return next;
    }
    *mult /= 10;
    return this->state_;
}

RMCParser::State RMCParser::add_to_min(char c, float * min_val, float * mult, RMCParser::State next) {
    if (!digit(c) && c != '.' && c != ',') {
        // Invalid
        return WaitForHeader;
    } 
    if (c == '.') {
        // skip
        return this->state_;
    }
    if (c != ',') {
        *min_val += (c - '0') * (*mult);
        *mult /= 10.0;
        return this->state_;
    }
    return next;
}

RMCParser::State RMCParser::add_dir(char c, int * deg_val, float * min_val, RMCParser::State next) {
    if (c != 'N' && c != 'S' && c != 'E' && c != 'W' && c != ',') {
        // Invalid
        return WaitForHeader;
    } else if (c == 'S' || c == 'W') {
        *deg_val *= -1;
        *min_val *= -1;
    } else if (c == ',') {
        this->lon_deg_mult_ = 100;
        return next;
    }
    return this->state_;
}


GSVParser::GSVParser() :
    state_(WaitForHeader),
    header_type_offset_(0),
    num_sats_(0)
{
}

// TODO we can totally factor out most of these states
bool GSVParser::feed(char c) {
    State next_state = this->state_;
    bool completed = false;
    switch (this->state_) {
        case WaitForHeader:
            if (c == '$') {
                this->header_type_offset_ = 0;
                memset(this->header_type_, 0, sizeof(this->header_type_));
                this->num_sats_ = 0;
                next_state = ReadingHeader;
            }
            break;
        case ReadingHeader:
            if (c == ',' || this->header_type_offset_ >= 5) {
                if (this->header_type_offset_ < 5) {
                    // Invalid, wait for another message.
                    next_state = WaitForHeader;
                } else if (this->header_type_[2] == 'G' &&
                        this->header_type_[3] == 'S' &&
                        this->header_type_[4] == 'V') {
                    next_state = NumMessages;
                } else {
                    next_state = WaitForHeader;
                }
            } else if (alpha(c)) {
                if (this->header_type_offset_ >= HEADER_TYPE_LEN) {
                    // Invalid
                    next_state = WaitForHeader;
                } else {
                    this->header_type_[this->header_type_offset_++] = c;
                }
            } else {
                next_state = WaitForHeader;
            }
            break;
        case NumMessages:
            if (c == ',') {
                next_state = MessageIndex;
            } else {
                if (!digit(c)) {
                    // Invalid
                    next_state = WaitForHeader;
                }
            }
            break;
        case MessageIndex:
            if (c == ',') {
                this->num_sats_mult_ = 10;
                next_state = NumSatellites;
            } else {
                if (!digit(c)) {
                    // Invalid
                    next_state = WaitForHeader;
                }
            }
            break;
        case NumSatellites:
            if (c == ',') {
                next_state = WaitForHeader;
                completed = true;
            } else if (digit(c)) {
                this->num_sats_ += (c - '0') * this->num_sats_mult_;
                this->num_sats_mult_ /= 10;
            } else {
                // Invalid
                next_state = WaitForHeader;
            }
            break;
    }
    this->state_ = next_state;
    return completed;
}
