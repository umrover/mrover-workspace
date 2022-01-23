#include <iostream>
#include "I2C.h"
#include <vector>
#include <thread>

# define M_PI           3.14159265358979323846  /* pi */

#define OFF         0x00,   0,  0
#define ON          0x0F,   0,  0
#define OPEN        0x10,   2,  0
#define OPEN_PLUS   0x1F,   4,  4
#define CLOSED      0x20,   8,  0
#define CLOSED_PLUS 0x2F,   8,  4
#define CONFIG_PWM  0x30,   2,  0
#define CONFIG_K    0x3F,   12, 0 // maybe rename to CONFIG_KPID since its all 3
#define GET_K       0x5F,   0,  12
#define QUAD        0x40,   0,  4
#define ADJUST      0x4F,   4,  0
#define ABS_ENC     0x50,   0,  4
#define LIMIT       0x60,   0,  1

#define UINT8_POINTER_T reinterpret_cast<uint8_t *>

using namespace std;
int get_addr(int nucleo, int channel = 0);
void off(int addr);
void on(int addr);
float openPlus(int addr, float speed);







int get_addr(int nucleo, int channel = 0)
{
    return (nucleo << 4) | channel;
}

void off(int addr)
{
    try
    {
        I2C::transact(addr, OFF, nullptr, nullptr);
        printf("test off transaction successful on slave %i \n", addr);
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test off failed on slave %i \n", addr);
    }
}

void on(int addr)
{
    try
    {
        I2C::transact(addr, ON, nullptr, nullptr);
        printf("test on transaction successful on slave %i \n", addr);
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test on failed on slave %i \n", addr);
    }
}

float openPlus(int addr, float speed)
{
    try
    {
        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&speed), 4);
        int32_t raw_angle;

        I2C::transact(addr, OPEN_PLUS, buffer, UINT8_POINTER_T(&raw_angle));

        int joint = (addr & 0b1) + (((addr >> 4) - 1) * 2); 
        float rad_angle = (raw_angle / cpr[joint]) * 2 * M_PI;
        float deg_angle = (raw_angle / cpr[joint]) * 360; 

        printf("test open plus transaction successful on slave %i, raw quad: %i, cpr: % f, in radians: %f \n", addr, raw_angle, cpr[joint], deg_angle);
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test open plus failed on slave %i \n", addr);
        return 0;
    }
}

// untested
float closedPlus(int addr, float target)
{
    try
    {
        uint8_t buffer[8];
        float feed_forward = 0;
        memcpy(buffer, UINT8_POINTER_T(&feed_forward), 4);

        int joint = (addr & 0b1) + (((addr >> 4) - 1) * 2); 
        int32_t target_ticks = (target / (2 * M_PI)) * cpr[joint];
        memcpy(buffer + 4, UINT8_POINTER_T(&target), 4); 
       
        float raw_angle;
        I2C::transact(addr, CLOSED_PLUS, buffer, UINT8_POINTER_T(&raw_angle));

        float rad_angle = (raw_angle / cpr[joint]) * 2 * M_PI;
        printf("test closed plus transaction successful on slave %i, %f \n", addr, rad_angle);
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test closed plus failed on slave %i \n", addr);
        return 0;
    }
}