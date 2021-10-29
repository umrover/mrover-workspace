#include <iostream>
#include "I2C.h"

#define OFF         0x00,   0,  0
#define ON          0x0F,   0,  0
#define OPEN        0x10,   2,  0
#define OPEN_PLUS   0x1F,   4,  0
#define CLOSED      0x20,   8,  0
#define CLOSED_PLUS 0x2F,   8,  4
#define CONFIG_PWM  0x30,   2,  0
#define CONFIG_K    0x3F,   12, 0
#define QUAD        0x40,   0,  4
#define ADJUST      0x4F,   4,  0
#define ABS_ENC     0x50,   0,  2
#define LIMIT       0x60,   0,  1

using namespace std;

// TODO might fix this later
// unsure about the 0 (channel)
#define ADDR ((2 << 4) | 0)

#define PRINT_TEST_START printf("Running Test #%2d, %s\n", ++num_tests_ran, __FUNCTION__);
#define PRINT_TEST_END printf("Finished Test #%2d, %s\n\n", num_tests_ran, __FUNCTION__);

int num_tests_ran = 0;

void test_ON_function();

int main()
{
    I2C::init();

    test_ON_function();

}


void test_ON_function()
{
    PRINT_TEST_START
    I2C::transact(ADDR, ON, nullptr, nullptr);
    PRINT_TEST_END
}
