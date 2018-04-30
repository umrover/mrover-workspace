
#ifndef PINS_H
#define PINS_H

// Pin names for development board
const PinName LED1_DEV = PinName(LED1);
const PinName LED2_DEV = PinName(LED2);

const PinName I2C_SDA_DEV = PinName(PB_7);
const PinName I2C_SCL_DEV = PinName(PB_6);

const PinName UART_TX_DEV = PinName(PA_0);
const PinName UART_RX_DEV = PinName(PA_1);

const PinName ANALOGIN1_DEV = PinName(PA_0);
const PinName ANALOGIN2_DEV = PinName(PA_1);
const PinName ANALOGIN3_DEV = PinName(PA_2);


// Pin names for custom MRover board
const PinName LED1_PROD = PinName(PB_12);       // p51

const PinName I2C_SDA_PROD = PinName(PB_7);     // p93
const PinName I2C_SCL_PROD = PinName(PB_8);     // p95

const PinName UART_TX1_PROD = PinName(PA_9);    // p68
const PinName UART_RX1_PROD = PinName(PA_10);   // p69
const PinName UART_TX2_PROD = PinName(PB_10);   // p47
const PinName UART_RX2_PROD = PinName(PB_11);   // p48

const PinName ANALOGIN1_PROD = PinName();       // p
const PinName ANALOGIN2_PROD = PinName();       // p
const PinName ANALOGIN3_PROD = PinName();       // p

const PinName CAN_TX_PROD = PinName(PA_12);     // p71
const PinName CAN_RX_PROD = PinName(PA_11);     // p70

#endif