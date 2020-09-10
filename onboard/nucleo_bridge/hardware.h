#ifndef HARDWARE_H
#define HARDWARE_H

#include <algorithm>

enum Type
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
    uint16_t pwmMin, pwmMax, pwmPeriod;
    Type type;

    Hardware() : type(None) {}

    Hardware(Type type) : type(type)
    {
        switch (type)
        {
        case Talon24V:
            pwmMin = 1000;
            pwmMax = 2000;
            pwmPeriod = 3000;
            break;
        case Talon12V:
            pwmMin = 1250;
            pwmMax = 1750;
            pwmPeriod = 3000;
            break;
        case Talon6V:
            pwmMin = 1375;
            pwmMax = 1625;
            pwmPeriod = 3000;
            break;
        case HBridgePos:
        case HBridgeNeg:
            pwmMin = 0;
            pwmMax = 3000;
            pwmPeriod = 3000;
            break;
        case None:
            break;
        }
    }

    //Helper function that takes a [-1.0, 1.0] (reranged between min and max) input and converts it into a 16-bit pwm output
    float rerange(float input, float min, float max)
    {
        return (((pwmMax - pwmMin) / (max - min)) * (input - min)) + pwmMin;
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
        case None:
            return 0;
        }
        return 0;
    }
};

#endif
