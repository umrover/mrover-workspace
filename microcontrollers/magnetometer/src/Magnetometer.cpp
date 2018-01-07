#include "mbed.h"

DigitalIn  mypin(p1); // change this
CAN can1(p9, p10);


int main() {
    // check mypin object is initialized and connected to a pin
    if(!mypin.is_connected()) {
        printf("Error with connection");
    }

    while(1) {
        auto in = mypin.read();
        CANMessage out;
        out.format = CANStandard;
        out.len = 1;
        out.data[1] = in;
        can1.write(out);
    }
}
