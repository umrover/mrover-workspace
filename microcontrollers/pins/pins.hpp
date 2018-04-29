
#ifndef PINS_H
#define PINS_H

// Pin names for development board
const PinName LED1_DEV = PinName(LED1);
const PinName LED2_DEV = PinName(LED2);
const PinName IMU_I2C_SDA_DEV = PinName(PB_7);
const PinName IMU_I2C_SCL_DEV = PinName(PB_6);
const PinName RTK_UART_TX_DEV = PinName(PA_0);
const PinName RTK_UART_RX_DEV = PinName(PA_1);

// Pin names for custom MRover board
const PinName LED1_PROD = PinName(PB_12);
const PinName IMU_I2C_SDA_PROD = PinName(PB_7);
const PinName IMU_I2C_SCL_PROD = PinName(PB_8);
const PinName RTK_UART_TX1_PROD = PinName(PA_9);
const PinName RTK_UART_RX1_PROD = PinName(PA_10);
const PinName RTK_UART_TX2_PROD = PinName(PB_10);
const PinName RTK_UART_RX2_PROD = PinName(PB_11);
const PinName CAN_TX_PROD = PinName(PA_12);
const PinName CAN_RX_PROD = PinName(PA_11);

#endif