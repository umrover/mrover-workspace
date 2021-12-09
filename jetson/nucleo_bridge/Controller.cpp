#include "Controller.h"

//Wrapper for I2C transact, autofilling the i2c address of the Controller by using ControllerMap::get_i2c_address()
void Controller::transact(uint8_t cmd, uint8_t write_num, uint8_t read_num, uint8_t *write_buf, uint8_t *read_buf)
{
    I2C::transact(ControllerMap::get_i2c_address(name), cmd, write_num, read_num, write_buf, read_buf);
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
        // turn on 
        transact(ON, nullptr, nullptr);

        uint8_t buffer[32];
        //buffer sends max percentage speed  
        memcpy(buffer, UINT8_POINTER_T(&(hardware.speed_max)), 2);
        transact(CONFIG_PWM, buffer, nullptr);

        // config kp, ki, pd
        memcpy(buffer, UINT8_POINTER_T(&(kP)), 4);
        memcpy(buffer + 4, UINT8_POINTER_T(&(kI)), 4);
        memcpy(buffer + 8, UINT8_POINTER_T(&(kD)), 4);
        transact(CONFIG_K, buffer, nullptr);

        // get absolute encoder correction #
        float abs_raw_angle = 0;;
        transact(ABS_ENC, nullptr, UINT8_POINTER_T(&(abs_raw_angle)));

        // get value in quad counts adjust quadrature encoder 
        int32_t adjusted_quad = (abs_raw_angle / (2 * M_PI)) * quad_cpr);
        memcpy(buffer, UINT8_POINTER_T(&(adjusted_quad)), 4);
        transact(ADJUST,buffer, nullptr);

        ControllerMap::make_live(name);
    }
    catch (IOFailure &e)
    {
        printf("activate failed on %s\n", name.c_str());
        throw IOFailure();
    }
}

//Helper function to convert raw angle to radians. Also checks if new angle is close to old angle <depreciated>
void Controller::record_angle(int32_t raw_angle)
{
    if (name == "RA1")
    {
        float abs_raw_angle = 0;;
        transact(ABS_ENC, nullptr, UINT8_POINTER_T(&(abs_raw_angle)));
        // record absolute value 
        current_angle = abs_raw_angle;            
    }
    else 
    {
        // record quadrature 
        current_angle = (raw_angle / quad_cpr) * 2 * M_PI;
    } 
}

//Initialize the Controller. Need to know which nucleo and which channel on the nucleo to use
Controller::Controller(std::string name, std::string type) : name(name), hardware(Hardware(type)){}

//Handles an open loop command with input [-1.0, 1.0], scaled to PWM limits
void Controller::open_loop(float input)
{
    try
    {
        make_live();

        uint8_t buffer[4];
        float speed = hardware.throttle(input);
        memcpy(buffer, UINT8_POINTER_T(&speed), 4);
    
        int32_t raw_angle;
        transact(OPEN_PLUS, buffer, UINT8_POINTER_T(&raw_angle));

        // handles if joint B 
        record_angle(raw_angle);       
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

        float feed_forward = 0; //torque * torque_scale;
        int32_t closed_setpoint = static_cast<int32_t>((angle / (2.0 * M_PI)) * quad_cpr);
        uint8_t buffer[32];
        int32_t angle;
        memcpy(buffer, UINT8_POINTER_T(&feed_forward), 4);
        memcpy(buffer + 4, UINT8_POINTER_T(&closed_setpoint), 4);
        transact(CLOSED_PLUS, buffer, UINT8_POINTER_T(&angle));

        // handles if is joint B
        record_angle(angle);
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
            memcpy(buffer, UINT8_POINTER_T(&KP), 4);
            memcpy(buffer + 4, UINT8_POINTER_T(&KI), 4);
            memcpy(buffer + 8, UINT8_POINTER_T(&KD), 4);
            transact(CONFIG_K, buffer, nullptr);
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
            transact(ADJUST, UINT8_POINTER_T(&zero), nullptr);
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
        transact(QUAD, nullptr, UINT8_POINTER_T(&angle));
        
        // handles if joint B
        record_angle(angle);
    }
    catch (IOFailure &e)
    {
        printf("angle failed on %s\n", name.c_str());
    }
}
