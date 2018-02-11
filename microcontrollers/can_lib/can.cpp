#include <mbed.h>
#include "can.hpp"

float fxp_10_22_to_float(int fxp) {
    float CONVERSION_CONST = 0.0000002384185791015625;
    float raw = float(fxp);
    return raw * CONVERSION_CONST;
}


int float_to_fxp_10_22(float val) {
    float CONVERSION_CONST = float(0x400000);
    return int(val * CONVERSION_CONST);
}


int send_msg(CAN can, int arbId, int counter, float data, int bytes_to_send) {
    // Encode msg into CAN data format
    int encoded_data = float_to_fxp_10_22(data);

    char msg[8];
    memcpy(&msg[0], &encoded_data, sizeof(int));
    memcpy(&msg[4], &counter, sizeof(int));

    // Send the message
    if(can.write(CANMessage(arbId, msg, bytes_to_send))) { // wrote successfully
        return 0;
    }
    else { // failed to write
        return 1;
    }
}


int recv_msg(CAN can, float &message_data) {
    CANMessage msg;
    if (can.read(msg)) { // read successfully
        int data;
        memcpy(&data, &msg.data, sizeof(int));
        message_data = fxp_10_22_to_float(data);
        return 0;
    }
    else { // failed to read
        return 1;
    }
}
