#include "frame_serial.hpp"

const uint8_t START = 0x12;
const uint8_t ESC = 0x7D;
const uint8_t END = 0x13;

void write_frame(Serial &s, const uint8_t *buf, size_t len) {
    s.putc(START);
    for(uint8_t i = 0; i < len; ++i) {
        if(buf[i] == ESC || buf[i] == END) {
            s.putc(ESC);
        }
        s.putc(buf[i]);
    }
    s.putc(END);
}
