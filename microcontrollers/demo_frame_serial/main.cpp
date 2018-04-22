#include <mbed.h>
#include "frame_serial.hpp"

Serial s(PA_9, PA_10, 115200);

int main() {
    uint8_t frame1[12] = "hello world";
    uint8_t frame2[8] = "hi milo";
    uint8_t frame3[10] = "hi justin";
    
    while(true) {
        write_frame(s, frame1, 12);
        wait(1);
        write_frame(s, frame2, 8);
        wait(1);
        write_frame(s, frame3, 10);
        wait(1);
    }
}
