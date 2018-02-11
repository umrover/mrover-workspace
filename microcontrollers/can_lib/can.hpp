#include <mbed.h>

float fxp_10_22_to_float(int fxp);

int float_to_fxp_10_22(float val);

int send_msg(CAN can, int arbId, int counter, float data, int bytes_to_send=8);

int recv_msg(CAN can, float &message_data);

