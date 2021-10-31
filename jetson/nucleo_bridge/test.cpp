#include <iostream>
#include "I2C.h"
#include <vector>
#include <thread>

#define OFF         0x00,   0,  0
#define ON          0x0F,   0,  0
#define OPEN        0x10,   2,  0
#define OPEN_PLUS   0x1F,   4,  0
#define CLOSED      0x20,   8,  0
#define CLOSED_PLUS 0x2F,   8,  4
#define CONFIG_PWM  0x30,   2,  0
#define CONFIG_K    0x3F,   12, 0 // maybe rename to CONFIG_KPID since its all 3
#define GET_K       0x5F,   0,  12
#define QUAD        0x40,   0,  4
#define ADJUST      0x4F,   4,  0
#define ABS_ENC     0x50,   0,  2
#define LIMIT       0x60,   0,  1

#define UINT8_POINTER_T reinterpret_cast<uint8_t *>

using namespace std;

// TODO might fix this later
// unsure about the 0 (channel)
// #define ADDR ((2 << 4) | 0)

#define PRINT_TEST_START printf("Running Test #%2d, %s\n", ++num_tests_ran, __FUNCTION__);
#define PRINT_TEST_END printf("Finished Test #%2d, %s\n\n", num_tests_ran, __FUNCTION__);

int num_tests_ran = 0;

std::vector<int> i2c_address;

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
        printf("test off failed on slave %i \n", addr);
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
        printf("test on failed on slave %i \n", addr);
    }
}

void openPlus(int addr, float speed)
{
    try
    {
        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&speed), 4);

        printf("test open plus transaction successful on slave %i \n", addr);
        return;
    }
    catch (IOFailure &e)
    {
        printf("test open plus failed on slave %i \n", addr);
        return;
    }
}

void configPWM(int addr, int max_speed)
{
    try
    {
        uint8_t buffer[2];
        memcpy(buffer, UINT8_POINTER_T(&(max_speed)), 2);

        I2C::transact(addr, CONFIG_PWM, buffer, nullptr);
        printf("test config pwm transaction successful on slave %i \n", addr);
    }
    catch (IOFailure &e)
    {
        printf("test config pwm failed on slave %i \n", addr);
    }
}

void setKPID(int addr, int p, int i, int d)
{
    try
    {
        uint8_t buffer[12];
        memcpy(buffer + 0, UINT8_POINTER_T(&p), 4);
        memcpy(buffer + 4, UINT8_POINTER_T(&i), 4);
        memcpy(buffer + 8, UINT8_POINTER_T(&d), 4);
        I2C::transact(addr, CONFIG_K, buffer, nullptr);
        printf("test set kpid transaction successful on slave %i \n", addr);
    }
    catch (IOFailure &e)
    {
        printf("test set kpid failed on slave %i \n", addr);
    }
}

//void getKPID(int addr)
//{
//    try
//    {
//        uint8_t buffer[24];
//        I2C::transact(addr, GET_K, nullptr, buffer);
//        int p, i, d;
//        memcpy(UINT8_POINTER_T(&p), buffer + 0, 4);
//        memcpy(UINT8_POINTER_T(&i), buffer + 4, 4);
//        memcpy(UINT8_POINTER_T(&d), buffer + 8, 4);
//        printf("test get kpid transaction successful on slave %i \n", addr);
//    }
//    catch (IOFailure &e)
//    {
//        printf("test get kpid failed on slave %i \n", addr);
//    }
//}

uint32_t quadEnc(int addr)
{
    try
    {
        uint32_t raw_angle;
        I2C::transact(addr, QUAD, nullptr, UINT8_POINTER_T(&(raw_angle)));
        printf("test quad transaction successful on slave %i \n", addr);
        return raw_angle;
    }
    catch (IOFailure &e)
    {
        printf("test quad transaction failed on slave %i \n", addr);
        return 0;
    }
}

uint32_t absEnc(int addr)
{
    try
    {
        uint32_t abs_raw_angle;
        I2C::transact(addr, ABS_ENC, nullptr, UINT8_POINTER_T(&(abs_raw_angle)));
        printf("test abs quad transaction successful on slave %i \n", addr);
        return abs_raw_angle;
    }
    catch (IOFailure &e)
    {
        printf("test abs quad transaction failed on slave %i \n", addr);
        return 0;
    }
}

//tests
// test functions for all slaves --> transactions are successful 
// full system test: off --> running --> off 
// good quad value test (i.e we run in and the value changes)
// test that we dont set over max 

void testOff()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        // test off 
        off(address);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testOn()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        // test off 
        on(address);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testConfigPWM()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        // test off 
        configPWM(address, 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testQuadEnc()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        // test off 
        uint32_t raw_angle = quadEnc(address);
        printf("raw angle: %i \n", raw_angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testAbsEnc()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        // test off
        uint32_t raw_angle = absEnc(address);
        printf("raw abs angle: %i \n", raw_angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testSetKPID()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        setKPID(address, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    PRINT_TEST_END
}

void testOpenPlus()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        // sets speed to half speed 
        openPlus(address, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    PRINT_TEST_END
}

void allDevBoardFunctions(int address)
{
    PRINT_TEST_START
    printf("turning rib %i off", address);
    off(address);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("turning rib %i on", address);
    on(address);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("setting pwm max to 0.75");
    configPWM(address, 75);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("turning setting speed to 0.5");
    openPlus(address, 0.5);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    printf("turning setting speed to 0");
    openPlus(address, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("turning rib %i off", address);
    off(address);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    PRINT_TEST_END
}

void testAllDevBoardFunctions()
{
    PRINT_TEST_START
    for (auto address: i2c_address)
    {
        allDevBoardFunctions(address);
    }
    PRINT_TEST_END
}

void testMax(int address)
{
    PRINT_TEST_START
    printf("setting max to 1.0");
    configPWM(address, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("setting speed to 0.5");
    openPlus(address, 50);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    printf("setting max to 0.5");
    configPWM(address, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("trying to set speed to 0.75 (should not change)");
    openPlus(address, 75);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    printf("setting speed to 0.0");
    openPlus(address, 0);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    PRINT_TEST_END
}

int main()
{
    for (int i = 1; i < 4; ++i)
    {
        i2c_address.push_back(get_addr(i, 0));
        i2c_address.push_back(get_addr(i, 1));
    }
    I2C::init();
    // testing if transactions are successful 
    testOff();
    testOn();
    testConfigPWM();
    // testQuadEnc();
//    testAbsEnc();
    testOpenPlus();

    testSetKPID();

    // // functionality tests
    // testAllFunctions(); 
    // testMax(i2c_address[0]);
    return 0;
}
