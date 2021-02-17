#ifndef HARDWARE_H
#define HARDWARE_H

#include <algorithm>

//Helper enum representing the valid types of real motor controllers
enum HardwareType
{
    Motor6V,
    Motor9V,
    Motor12V,
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
        if (input == "Motor6V") {
            return Motor6V;
        }
        else if (input == "Motor9V") {
            return Motor9V;
        }
        else if(input == "Motor12V") {
            return Motor12V;
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
        case Motor6V:
            speed_max = 16;           
            break;
        case Motor9V:
            speed_max = 25;
            break;
        case Motor12V:
            speed_max = 33;
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
