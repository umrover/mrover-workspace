#include "Controller.h"

//Wrapper for backend->i2c_transact, autofilling the i2c address of the Controller
void Controller::transact(uint8_t cmd, uint8_t writeNum, uint8_t readNum, uint8_t *writeBuf, uint8_t *readBuf)
{
    I2C::transact(ControllerMap::get_i2c_address(name), cmd, writeNum, readNum, writeBuf, readBuf);
}

//If this Controller is not live, make it live by configuring the real controller
void Controller::make_live()
{
    if (ControllerMap::check_if_live(name))
    {
        return;
    }

    try
    {
        uint8_t buffer[32];
        memcpy(buffer, POINTER(&(hardware.pwmMin)), 2);
        memcpy(buffer + 2, POINTER(&(hardware.pwmMax)), 2);
        memcpy(buffer + 4, POINTER(&(hardware.pwmPeriod)), 2);
        transact(CONFIG_PWM, buffer, nullptr);

        memcpy(buffer, POINTER(&(kP)), 4);
        memcpy(buffer + 4, POINTER(&(kI)), 4);
        memcpy(buffer + 8, POINTER(&(kD)), 4);
        transact(CONFIG_K, buffer, nullptr);

        uint16_t input = 0;
        //transact(SPI, nullptr, POINTER(&input));

        int32_t angle = static_cast<int32_t>(quadCPR * ((static_cast<float>(input) / spiCPR) + (startAngle / (2.0 * M_PI))));
        transact(ADJUST, POINTER(&angle), nullptr);

        transact(ON, nullptr, nullptr);

        ControllerMap::make_live(name);
    }
    catch (IOFailure &e)
    {
        printf("activate failed on %s\n", name.c_str());
        throw IOFailure();
    }
}

//Helper function to convert raw angle to radians. Also checks if new angle is close to old angle
void Controller::recordAngle(int32_t angle)
{
    float otherAngle = (static_cast<float>(angle) / quadCPR) * 2.0 * M_PI;
    if (std::abs(otherAngle - currentAngle) >= M_PI / 16.0)
    {
        return;
    }
    currentAngle = otherAngle;
}

//Initialize the Controller. Need to know which nucleo and which channel on the nucleo to use
Controller::Controller(Hardware inHardware) : hardware(inHardware){}

//Handles an open loop command with input [-1.0, 1.0], scaled to PWM limits
void Controller::open_loop(float input)
{
    try
    {
        make_live();

        uint16_t throttle = hardware.throttle(input);
        int32_t angle;
        transact(OPEN_PLUS, POINTER(&throttle), POINTER(&angle));

        recordAngle(angle);
    }
    catch (IOFailure &e)
    {
        printf("open loop failed on %s\n", name.c_str());
    }
}

//Sends a closed loop command with target angle in radians and optional precalculated torque in Nm
void Controller::closed_loop(float torque, float angle)
{
    try
    {
        make_live();

        float feedForward = 0; //torque * torqueScale;
        int32_t closedSetpoint = static_cast<int32_t>((angle / (2.0 * M_PI)) * quadCPR);
        uint8_t buffer[32];
        int32_t angle;
        memcpy(buffer, POINTER(&feedForward), 4);
        memcpy(buffer + 4, POINTER(&closedSetpoint), 4);
        transact(CLOSED_PLUS, buffer, POINTER(&angle));

        recordAngle(angle);
    }
    catch (IOFailure &e)
    {
        printf("closed loop failed on %s\n", name.c_str());
    }
}

//Sends a config command with PID inputs
void Controller::config(float KP, float KI, float KD)
{
    for (int attempts = 0; attempts < 100; ++attempts)
    {
        try
        {
            make_live();

            uint8_t buffer[32];
            memcpy(buffer, POINTER(&KP), 4);
            memcpy(buffer + 4, POINTER(&KI), 4);
            memcpy(buffer + 8, POINTER(&KD), 4);
            transact(CONFIG_K, buffer, nullptr);

            return;
        }
        catch (IOFailure &e)
        {
            printf("config failed on %s\n", name.c_str());
        }
    }
}

//Sends a zero command
void Controller::zero()
{
    for (int attempts = 0; attempts < 100; ++attempts)
    {
        try
        {
            make_live();

            int32_t zero = 0;
            transact(ADJUST, POINTER(&zero), nullptr);

            return;
        }
        catch (IOFailure &e)
        {
            printf("zero failed on %s\n", name.c_str());
        }
    }
}

//Sends a get angle command
void Controller::angle()
{
    if (!ControllerMap::check_if_live(name))
    {
        return;
    }

    try
    {
        int32_t angle;
        transact(QUAD, nullptr, POINTER(&angle));
        recordAngle(angle);
    }
    catch (IOFailure &e)
    {
        printf("angle failed on %s\n", name.c_str());
    }
}