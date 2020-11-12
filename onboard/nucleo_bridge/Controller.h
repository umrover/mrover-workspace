#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstring>
#include <vector>
#include <cmath>
#include <mutex>
#include <limits>
#include "Hardware.h"
#include "I2C.h"
#include "ControllerMap.h"

#define OFF 0x00, 0, 0
#define ON 0x0F, 0, 0
#define OPEN 0x10, 2, 0
#define OPEN_PLUS 0x1F, 2, 4
#define CLOSED 0x20, 8, 0
#define CLOSED_PLUS 0x2F, 8, 4
#define CONFIG_PWM 0x30, 6, 0
#define CONFIG_K 0x3F, 12, 0
#define QUAD 0x40, 0, 4
#define ADJUST 0x4F, 4, 0
#define SPI 0x50, 0, 2
#define LIMIT 0x60 0, 1

#define POINTER reinterpret_cast<uint8_t *>

class Controller {
public:
    float startAngle = 0.0;
    float torqueScale = 1.0;
    float quadCPR = std::numeric_limits<float>::infinity();
    float spiCPR = std::numeric_limits<float>::infinity();
    float currentAngle = 0.0;
    float kP, kI, kD = 0.0;

    std::string name;

    void recordAngle(int32_t angle);

private:
    Hardware hardware;

    //Wrapper for backend->i2c_transact, autofilling the i2c address of the Controller
    void transact(uint8_t cmd, uint8_t writeNum, uint8_t readNum, uint8_t *writeBuf, uint8_t *readBuf);

    //If this Controller is not live, make it live by configuring the real controller
    void make_live();

public:
    //Initialize the Controller. Need to know which type of hardware to use
    Controller(std::string name, std::string type);

    //Handles an open loop command with input [-1.0, 1.0], scaled to PWM limits
    void open_loop(float input);
    
    //Sends a closed loop command with target angle in radians and optional precalculated torque in Nm
    void closed_loop(float torque, float angle);

    //Sends a config command with PID inputs
    void config(float KP, float KI, float KD);

    //Sends a zero command
    void zero();

    //Sends a get angle command
    void angle();
};

#endif
