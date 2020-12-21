#ifndef HARDWARE_H
#define HARDWARE_H

#include <algorithm>

//Helper enum representing the valid types of real motor controllers
enum HardwareType
{
    Talon24V,
    Talon12V,
    Talon6V,
    HBridgePos,
    HBridgeNeg,
    None
};

//Helper class to abstract away Motor Controller details 
class Hardware
{
public:
    uint16_t pwm_min, pwm_max, pwm_period;
    HardwareType type;

    HardwareType getType(std::string input) 
    {
        if (input == "Talon24V")
        {
            return Talon24V;
        }
        else if (input == "Talon12V")
        {
            return Talon12V;
        }
        else if (input == "Talon6V")
        {
            return Talon6V;
        }
        else if (input == "HBridgePos")
        {
            return HBridgePos;
        }
        else if (input == "HBridgeNeg")
        {
            return HBridgeNeg;
        }
        else 
        {
            return None;
        }
    }

    Hardware() : type(None) {}

    Hardware(std::string input) : type(getType(input))
    {
        switch (type)
        {
        case Talon24V:
            pwm_min = 1000;
            pwm_max = 2000;
            pwm_period = 3000;
            break;
        case Talon12V:
            pwm_min = 1250;
            pwm_max = 1750;
            pwm_period = 3000;
            break;
        case Talon6V:
            pwm_min = 1375;
            pwm_max = 1625;
            pwm_period = 3000;
            break;
        case HBridgePos:
        case HBridgeNeg:
            pwm_min = 0;
            pwm_max = 3000;
            pwm_period = 3000;
            break;
        case None:
            break;
        }
    }

    //Helper function that takes a [-1.0, 1.0] (reranged between min and max) input and converts it into a 16-bit pwm output
    float rerange(float input, float min, float max)
    {
        return (((pwm_max - pwm_min) / (max - min)) * (input - min)) + pwm_min;
    }

    //Turns a given [-1.0,1.0] throttle input to a 16-bit pwm output
    uint16_t throttle(float input)
    {
        switch (type)
        {
        case Talon24V:
        case Talon12V:
        case Talon6V:
            return static_cast<uint16_t>(rerange(input, -1, 1));
        case HBridgePos:
            return static_cast<uint16_t>(rerange(std::max(0.0f, input), 0, 1));
        case HBridgeNeg:
            return static_cast<uint16_t>(rerange(std::min(0.0f, input), 0, -1));
        default:
            return 0;
        }
    }
};

#endif
