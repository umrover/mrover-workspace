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

#define PRINT_TEST_START printf("Running Test #%2d, %s\n", ++num_tests_ran, __FUNCTION__);
#define PRINT_TEST_END printf("Finished Test #%2d, %s\n\n", num_tests_ran, __FUNCTION__);

// reductions per motor 
float cpr[6] = { -28800.0, 1.0, 155040.0, -81600.0, -81600.0, -9072.0 };

int invert[6] = {1, 1, 1, 1, 1, 1};

int num_tests_ran = 0;

std::vector<int> i2c_address;

int get_addr(int nucleo, int channel = 0)
{
    return (nucleo << 4) | channel;
}

void busy_sleep(int seconds)
{
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < start + std::chrono::seconds(seconds))
    {
    }
}

void sleep(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
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
        memcpy(buffer, UINT8_POINTER_T(&speed), sizeof(speed));
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

float closedPlus(int addr, float angle) // -pi to pi
{
    try
    {
        //printf("test closed plus on slave %i sending target angle %f\n", addr, angle);
        uint8_t buffer[12];

        float ff = 0;
        memcpy(buffer, UINT8_POINTER_T(&ff), sizeof(ff));
        
        int32_t raw_angle;

        int joint = (addr & 0b1) + (((addr >> 4) - 1) * 2);

        if (joint != 1)
        {
            raw_angle = static_cast<int32_t>((angle / (2.0 * M_PI)) * cpr[joint]);
        }
        else 
        {
            memcpy(&raw_angle, &angle, sizeof(angle));
        }

        printf("test closed plus on slave %i sending target angle %f, raw angle %i\n", addr, angle, raw_angle);
        memcpy(buffer + 4, UINT8_POINTER_T(&raw_angle), sizeof(raw_angle));
        
        I2C::transact(addr, CLOSED_PLUS, buffer, UINT8_POINTER_T(&raw_angle));
        float deg_angle = (raw_angle / cpr[joint]) * 360; 
        float rad_angle = (raw_angle / cpr[joint]) * 2 * M_PI; 

        if (joint == 1)
        {
            memcpy(&rad_angle, &raw_angle, sizeof(rad_angle));
        }
        printf("test closed plus transaction successful on slave %i, at angle raw: %i, rad: %f \n", addr, raw_angle, rad_angle);
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test closed plus failed on slave %i \n", addr);
        return 0;
    }
}

void configPWM(int addr, int max_speed)
{
    try
    {
        uint8_t buffer[2];
        memcpy(buffer, UINT8_POINTER_T(&(max_speed)), sizeof(max_speed));
        I2C::transact(addr, CONFIG_PWM, buffer, nullptr);
        printf("test config pwm transaction successful on slave %i \n", addr);
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test config pwm failed on slave %i \n", addr);
    }
}

void setKPID(int addr, float p, float i, float d)
{
    try
    {
        uint8_t buffer[12];
        memcpy(buffer + 0, UINT8_POINTER_T(&p), sizeof(p));
        memcpy(buffer + 4, UINT8_POINTER_T(&i), sizeof(i));
        memcpy(buffer + 8, UINT8_POINTER_T(&d), sizeof(d));
        I2C::transact(addr, CONFIG_K, buffer, nullptr);
        printf("test set kpid transaction successful on slave %i \n", addr);
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test set kpid failed on slave %i \n", addr);
    }
}

void getKPID(int addr)
{
    try
    {
        uint8_t buffer[12];
        I2C::transact(addr, GET_K, nullptr, buffer);
        int p, i, d;
        memcpy(UINT8_POINTER_T(&p), buffer + 0, sizeof(p));
        memcpy(UINT8_POINTER_T(&i), buffer + 4, sizeof(i));
        memcpy(UINT8_POINTER_T(&d), buffer + 8, sizeof(d));
        printf("test get kpid transaction successful on slave %i %d %d %d \n", addr, p, i, d);
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test get kpid failed on slave %i \n", addr);
    }
}

float quadEnc(int addr)
{
    try
    {
        int32_t raw_angle;
        I2C::transact(addr, QUAD, nullptr, UINT8_POINTER_T(&(raw_angle)));

        int joint = (addr & 0b1) + (((addr >> 4) - 1) * 2); 
        float rad_angle = (raw_angle / cpr[joint]) * 2 * M_PI;

        printf("test quad enc transaction successful on slave %i, raw quad: %i, cpr: % f, in radians: %f \n", addr, raw_angle, cpr[joint], rad_angle);
        
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test quad transaction failed on slave %i \n", addr);
        return 0;
    }
}

float absEnc(int addr)
{
    try
    {
        float abs_raw_angle = 0;;
        I2C::transact(addr, ABS_ENC, nullptr, UINT8_POINTER_T(&(abs_raw_angle)));
        printf("test abs transaction successful on slave %i, %f \n", addr, abs_raw_angle * (180/M_PI));
        return abs_raw_angle; // in radians 
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test abs transaction failed on slave %i \n", addr);
        return 0;
    }
}

void adjust(int addr, int joint)
{
    try
    {
        // gets initial angles
        float quad_angle = quadEnc(addr);
        float abs_angle = absEnc(addr);

        if (joint == 5)
        {
            abs_angle = M_PI;
        }

        printf("before adjust quadrature value: %f, absolute value: %f on slave %i\n", quad_angle, abs_angle, addr);

        int joint = (addr & 0b1) + (((addr >> 4) - 1) * 2); 

        int32_t adjusted_quad = (abs_angle / (2 * M_PI)) * cpr[joint];

        // adjust transaction 
        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&(adjusted_quad)), sizeof(adjusted_quad));
        I2C::transact(addr, ADJUST, buffer, nullptr);

        // checks values after
        quad_angle = quadEnc(addr);
        abs_angle = absEnc(addr);
        if (joint == 5)
        {
            abs_angle = M_PI;
        }

        printf("after adjust quadrature value: %f, absolute value: %f on slave %i\n", quad_angle, abs_angle, addr);

        if (std::abs(quad_angle - abs_angle) > .2)
        {
            printf("test adjust transaction numerically failed on slave %i \n\n", addr);
        }
        else
        {
            printf("test adjust transaction successful on slave %i \n\n", addr);
        }
    }
    catch (IOFailure &e)
    {
        printf("test adjust transaction failed on slave %i \n\n", addr);
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
    for (auto address : i2c_address)
    {
        // test off
        off(address);
        sleep(10);
    }
    PRINT_TEST_END
}

void testOn()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        // test off
        on(address);
        sleep(10);
    }
    PRINT_TEST_END
}

void testConfigPWM()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        // test off
        int max = 30;
        if (address < 17)
        {
            max = 16;
        }
        if (address == 16)
        {
            max = 16;
        }
        configPWM(address, max);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testQuadEnc()
{
    for (auto address : i2c_address)
    {
        // test off
        uint32_t raw_angle = quadEnc(address);
        printf("[%i] raw quad angle: %i \n", address, raw_angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void testAbsEnc()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        // test off
        float raw_angle = static_cast<float>(absEnc(address));
        printf("[%i] raw abs angle: %f \n", address, raw_angle);
        sleep(10);
    }
    PRINT_TEST_END
}

void testClosed()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        int joint = (address & 0b1) + (((address >> 4) - 1) * 2); 
        float p = 0.001; 
        float i  = 0.00;
        float d = 0; 

        if (joint == 1)
        {
            p = 1.0;
        }

        setKPID(address, p * invert[joint], i * invert[joint], d * invert[joint]);
        printf("joint %i, kp %f, ki, %f \n", joint, p * invert[joint], i * invert[joint] );
        sleep(10);
    }
    while (1)
    {
        for (auto address : i2c_address)
        {
            int joint = (address & 0b1) + (((address >> 4) - 1) * 2); 
            float angle = quadEnc(address);
            
            float target = angle - 0.15;
            if (joint == 1)
            {
                angle = absEnc(address);
                target = angle - 0.15;
            }

            do 
            {
                angle = closedPlus(address, target);
                sleep(20);
            }
            while(joint != 1 ? abs(angle - target) > 0.01 : abs(angle - target) > 0.05);

	        printf("arrived at position 1\n");
	        sleep(1000);

            angle = quadEnc(address);
            target = angle + 0.15;

            if (joint == 1)
            {
                angle = absEnc(address);
                target = angle + 0.15;
            }
            do 
            {
                angle = closedPlus(address, target);
                sleep(20);
            }
            while( joint != 1 ? abs(angle - target) > 0.01 : abs(angle - target) > 0.05 );
	        printf("arrived at position 2\n");
	        sleep(1000);

            angle = quadEnc(address);
            if (joint == 1)
            {
                angle = absEnc(address);
            }
            target = angle - 0.25;
            do 
            {
                angle = closedPlus(address, target);
                sleep(20);
            }
            while( joint != 1 ? abs(angle - target) > 0.01 : abs(angle - target) > 0.05 );
	        printf("arrived at position 3\n");
	        sleep(1000);

            angle = quadEnc(address);
            if (joint == 1)
            {
                angle = absEnc(address);
            }
            target = angle + 0.25;
            do 
            {
                angle = closedPlus(address, target);
                sleep(20);
            }
            while( joint != 1 ? abs(angle - target) > 0.01 : abs(angle - target) > 0.05 );
	        printf("arrived at position 4\n");
	        sleep(1000);
        }
    }
    PRINT_TEST_END
}

void testAdjust()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        int joint = (address & 0b1) + (address >> 4);
        adjust(address, joint);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END   
}

void testSetKPID()
{
    PRINT_TEST_START 
    for (auto address : i2c_address)
    {
        getKPID(address);
        sleep(100);
    }
    PRINT_TEST_END
}

void testOpenPlus()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        float speed = 0.5f;
        if (address == 16)
        {
            speed = 0.25f;
        }

        for (int i = 0; i < 3; i++) {
            openPlus(address, speed);
            sleep(200);
        }
        for (int i = 0; i < 3; i++) {
            openPlus(address, -speed);
            sleep(200);
        }
        for (int i = 0; i < 5; i++) {
            openPlus(address, 0.0f);
            sleep(200);
        }
    }
    PRINT_TEST_END
}

void testOpenPlusWithAbs()
{
    PRINT_TEST_START
    for (auto address : i2c_address)
    {
        float speed = 0.5f;
        if (address == 16)
        {
            speed = 0.25f;
        }
        if (address == 32 || address == 33)
        {
            speed = 1.0f;
        }

        for (int i = 0; i < 6; i++) {
            openPlus(address, speed);
            sleep(200);
            absEnc(address);
            sleep(200);
        }
        for (int i = 0; i < 6; i++) {
            openPlus(address, 0.0f);
            sleep(200);
            absEnc(address);
            sleep(200);
        }
        for (int i = 0; i < 6; i++) {
            openPlus(address, -speed);
            sleep(200);
            absEnc(address);
            sleep(200);
        }
        for (int i = 0; i < 6; i++) {
            openPlus(address, 0.0f);
            sleep(200);
            absEnc(address);
            sleep(200);
        }
        std::cout << std::endl;
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
    for (auto address : i2c_address)
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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("setting max to 0.5");
    configPWM(address, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("trying to set speed to 0.75 (should not change)");
    openPlus(address, 75);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("setting speed to 0.0");
    openPlus(address, 0);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    PRINT_TEST_END
}

void testOnOff()
{
    for (auto addr : i2c_address)
    {
        // sets speed to half speed
        on(addr);
        sleep(20);
        off(addr);
        sleep(20);
    }
}

int main()
{
    for (int i = 1; i <= 3; ++i)
    {
        i2c_address.push_back(get_addr(i, 0));
        i2c_address.push_back(get_addr(i, 1));
        sleep(20);
    }
    I2C::init();
    testOn();
    testConfigPWM();
    testAdjust();

    while (1)
    {
        testClosed();
	    //testQuadEnc();
        //testOpenPlusWithAbs();
        //testOpenPlus();
        //testAbsEnc();
        sleep(100);

    }

    testOff();
    return 0;
}
