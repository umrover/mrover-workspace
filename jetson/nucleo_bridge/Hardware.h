#ifndef HARDWARE_H
#define HARDWARE_H

#include <algorithm>

//Helper enum representing the valid types of real motor controllers
enum HardwareType
{
    HBridge,
    Cytron,
    None
};

//Helper class to abstract away Motor Controller details 
class Hardware
{
public:
    uint16_t pwm_max;
    HardwareType type;

    HardwareType getType(std::string input) 
    {
        if (input == "HBridge") {
            return HBridge;
        }
        else if (input == "Cytron") {
            return Cytron;
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
        case HBridge:
            pwm_max = 100;           
            break;
        case Cytron:
            pwm_max = 100;
            break;
        case None:
            break;
        }
    }

    //Helper function that takes a [-1.0, 1.0] (reranged between min and max) input and converts it into a 16-bit pwm output
    // float rerange(float input, float min, float max)
    // {
    //     return (((pwm_max) / (max - min)) * (input - min));
    // }

    //Turns a given [-1.0,1.0] throttle input to a 16-bit pwm output
    uint16_t throttle(float input)
    {
        if (input > 1) {
            input = 1;
        }
        else if (input < -1) {
            input = -1;
        }
        // might not be needed
        switch (type)
        {
        case Cytron:
            return static_cast<int8_t>(input);
        case HBridge:
            return static_cast<int8_t>(input);
        default:
            return 0;
        }
    }
};

#endif
