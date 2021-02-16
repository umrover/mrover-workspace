#ifndef HARDWARE_H
#define HARDWARE_H

#include <algorithm>

//Helper enum representing the valid types of real motor controllers
enum HardwareType
{
    HBridge6V,
    HBridge9V,
    HBridge12V,
    Cytron,
    None
};

//Helper class to abstract away Motor Controller details 
class Hardware
{
public:
    uint16_t speed_max; //out of 100 to avoid sending floats
    HardwareType type;

    HardwareType getType(std::string input) 
    {
        if (input == "HBridge6V") {
            return HBridge6V;
        }
        else if (input == "HBridge9V") {
            return HBridge9V;
        }
        else if(input == "HBridge12V") {
            return HBridge12V;
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
        case HBridge6V:
            speed_max = 16;           
            break;
        case HBridge9V:
            speed_max = 25;
            break;
        case HBridge12V:
            speed_max = 33;
	        break;
        case Cytron:
            speed_max = 70;
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

    //limits throttle
    float throttle(float input)
    {
        if (input > 1.0) {
            input = 1.0;
        }
        else if (input < -1.0) {
            input = -1.0;
        }
        return input;
    }
};

#endif
