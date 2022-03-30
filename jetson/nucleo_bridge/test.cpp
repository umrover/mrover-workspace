#include <iostream>
#include "ControllerMap.h"
#include "Controller.h"
#include "Hardware.h"
#include "I2C.h"

#include <vector>
#include <thread>

using namespace std;

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

int num_tests_ran = 0;

std::vector<string> motor_names;

int get_addr(int nucleo, int channel = 0)
{
    return (nucleo << 4) | channel;
}

void sleep(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void off(string name)
{
    try
    {
        ControllerMap::controllers[name]->transact(OFF, nullptr, nullptr);
        printf("test off transaction successful on slave %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test off failed on slave %s \n", name.c_str());
    }
}

void on(string name)
{
    try
    {
        ControllerMap::controllers[name]->transact(ON, nullptr, nullptr);
        printf("test on transaction successful on slave %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test on failed on slave %s \n", name.c_str());
    }
}

void make_live(string name)
{
    try
    {
        ControllerMap::controllers[name]->make_live();
        printf("test on transaction successful on slave %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test on failed on slave %s \n", name.c_str());
    }
}

float openPlus(string name, float speed)
{
    try
    {
        ControllerMap::controllers[name]->open_loop(speed);
        float rad_angle = ControllerMap::controllers[name]->current_angle;
        float deg_angle = rad_angle / (2 * M_PI) * 360;
        float cpr_val = ControllerMap::controllers[name]->quad_cpr;

        printf("test openPlus transaction successful on slave %s, rad angle: %f, in degrees: %f \n", name.c_str(), rad_angle, deg_angle);
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test open plus failed on slave %s \n", name.c_str());
        return 0;
    }
}

float closedPlus(string name, float angle) // -pi to pi
{
    try
    {
        ControllerMap::controllers[name]->closed_loop(0, angle);
        float rad_angle = ControllerMap::controllers[name]->current_angle;
        float deg_angle = rad_angle/ (2 * M_PI) * 360;
        // TODO: TF is raw angle ????
        printf("test closed plus transaction successful on slave %s, at angle raw: %i, rad: %f \n", name.c_str(), raw_angle, rad_angle);
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test closed plus failed on slave %s \n", name.c_str());
        return 0;
    }
}

void configPWM(string name, int max_speed)
{
    try
    {
        uint8_t buffer[2];
        memcpy(buffer, UINT8_POINTER_T(&(max_speed)), sizeof(max_speed));
        I2C::transact(ControllerMap::get_i2c_address(name), CONFIG_PWM, buffer, nullptr);
        printf("test config pwm transaction successful on slave %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test config pwm failed on slave %s \n", name.c_str());
    }
}

void setKPID(string name, float p, float i, float d)
{
    try
    {
        ControllerMap::controllers[name]->config(p, i, d);
        printf("test set kpid transaction successful on slave %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test set kpid failed on slave %s \n", name.c_str());
    }
}

void getKPID(string name)
{
    try
    {
        uint8_t buffer[12];
        I2C::transact(ControllerMap::get_i2c_address(name), GET_K, nullptr, buffer);
        int p, i, d;
        memcpy(UINT8_POINTER_T(&p), buffer + 0, sizeof(p));
        memcpy(UINT8_POINTER_T(&i), buffer + 4, sizeof(i));
        memcpy(UINT8_POINTER_T(&d), buffer + 8, sizeof(d));
        // TODO: is it okay?
        printf("test get kpid transaction successful on slave %s %d %d %d \n", name.c_str(), p, i, d);
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test get kpid failed on slave %s \n", name.c_str());
    }
}

float quadEnc(string name)//check printing 
{
    try
    {
        // int32_t raw_angle;
        // I2C::transact(addr, QUAD, nullptr, UINT8_POINTER_T(&(raw_angle)));

        // int joint = (addr & 0b1) + (((addr >> 4) - 1) * 2); 
        // float rad_angle = (raw_angle / cpr[joint]) * 2 * M_PI;
        ControllerMap::controllers[name]->angle();
        float rad_angle=ControllerMap::controllers[name]->current_angle;
        float quad_cpr = ControllerMap::controllers[name]->quad_cpr;
       // printf("test quad enc transaction successful on slave %s, raw quad: %i, cpr: % f, in radians: %f \n", name.c_str(), raw_angle, cpr[joint], rad_angle);
        printf("test quad on slave %s, in radians: %f \n", name.c_str(), rad_angle);
        return rad_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test quad transaction failed on slave %s \n", name.c_str());
        return 0;
    }
}

//uint16_t absEnc(int addr)
float absEnc(string name)
{
    try
    {
        float angle = 0;
       ControllerMap::controllers[name]->transact(ABS_ENC, nullptr, UINT8_POINTER_T(&angle));
        printf("test abs on %s: degrees is %f \n", name.c_str(), angle * (180/M_PI));
        return angle; // in radians 
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "test abs failed on slave %s \n", name.c_str());
        return 0;
    }
}

void adjust(string name)
{
    try
    {
        // gets initial angles
        float quad_angle = quadEnc(name);
        float abs_angle = absEnc(name);

        if (name == "RA_5")
        {
            abs_angle = M_PI;
        }

        printf("before adjust quadrature value: %f, absolute value: %f on slave %s\n", quad_angle, abs_angle, name.c_str()); 

        // gets absolute angle in quadrature counts
        // quad_angle = (quad_angle / cpr[joint]) * (2 * M_PI); // counts to radians
        // abs_angle = static_cast<uint32_t>(abs_angle); // comes out in 'radians'

        int32_t adjusted_quad = (abs_angle / (2 * M_PI)) * ControllerMap::controllers[name]->quad_cpr;;

        // adjust transaction 
        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&(adjusted_quad)), sizeof(adjusted_quad));
        I2C::transact(ControllerMap::get_i2c_address(name), ADJUST, buffer, nullptr);
        //ControllerMap::controllers[name]->zero();

        // checks values after
        quad_angle = quadEnc(name);
        abs_angle = absEnc(name);
        if (name == "RA_5")
        {
            abs_angle = M_PI;
        }

        printf("after adjust quadrature value: %f, absolute value: %f on slave %s\n", quad_angle, abs_angle, name.c_str());

        if (std::abs(quad_angle - abs_angle) > .2)
        {
            printf("test adjust transaction numerically failed on slave %s \n\n", name.c_str());
        }
        else
        {
            printf("test adjust transaction successful on slave %s \n\n", name.c_str());
        }
    }
    catch (IOFailure &e)
    {
        printf("test adjust transaction failed on slave %s \n\n", name.c_str());
    }    
}

//tests
// test functions for all slaves --> transactions are successful
// full system test: off --> running --> off
// good quad value test (i.e we run in and the value changes)
// test that we dont set over max

void testMakeLive()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        // test off
        make_live(name);
        sleep(10);
    }
    PRINT_TEST_END
}

void testOff()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        // test off
        off(name);
        sleep(10);
    }
    PRINT_TEST_END
}

void testOn()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        // test off
        on(name);
        sleep(10);
    }
    PRINT_TEST_END
}

void testConfigPWM()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        // test off
        int max = 30;
        if (ControllerMap::get_i2c_address(name) < 17)
        {
            max = 16;
        }
        if (ControllerMap::get_i2c_address(name) == 16)
        {
            max = 16;
        }
        configPWM(name, max);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END
}

void testQuadEnc()
{
    for (auto name : motor_names)
    {
        // test off
        uint32_t raw_angle = quadEnc(name);
        printf("[%s] raw quad angle: %i \n", name.c_str(), raw_angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void testAbsEnc()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        // test off
        float raw_angle = static_cast<float>(absEnc(name));
        printf("[%s] raw abs angle: %f \n", name.c_str(), raw_angle);
        sleep(10);
    }
    PRINT_TEST_END
}

void testClosed()
{
    PRINT_TEST_START
    // testMakeLive() DOES ALL OF THIS ALREADY
    // for (auto name : motor_names)
    // {
    //     int joint = (ControllerMap::get_i2c_address(name) & 0b1) + (((ControllerMap::get_i2c_address(name) >> 4) - 1) * 2); 
    //     float p = 0.001; 
    //     float i  = 0.00;
    //     float d = 0; 

    //     if (joint == 1)
    //     {
    //         p = 1.0;
    //     }

    //     setKPID(name, p * invert[joint], i * invert[joint], d * invert[joint]);
    //     printf("joint %i, kp %f, ki, %f \n", joint, p * invert[joint], i * invert[joint] );
    //     sleep(10);
    // }
    for (auto name : motor_names)
    {
        int p, i, d;
        p = ControllerMap::controllers[name]->kP;
        i = ControllerMap::controllers[name]->kI;
        d = ControllerMap::controllers[name]->kD;
        setKPID(name, p, i, d);
        printf("joint %s, kp %f, ki, %f \n", name.c_str(), p, i);
        sleep(10);
    }

    while (1)
    {
        for (auto name : motor_names)
        {
            float angle = 0;
            float offset[4] = { 0.15, -0.15, 0.25, -0.25 };
            float target = 0;

            for (int i = 0; i < 4; ++i) {
                if (name == "RA_B" || name == "SA_B")
                {
                    angle = absEnc(name);
                }
                else {
                    angle = quadEnc(name);
                }

                target = angle + offset[i];

                do 
                {
                    angle = closedPlus(name, target);
                    sleep(20);
                }
                while( name != "RA_B" && name != "SA_B" ? abs(angle - target) > 0.01 : abs(angle - target) > 0.05 );

                printf("arrived at position %i\n", i);
                sleep(1000);
            }
        }
    }
    PRINT_TEST_END
}

void testAdjust()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        adjust(name);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    PRINT_TEST_END   
}

void testSetKPID()
{
    PRINT_TEST_START 
    for (auto name : motor_names)
    {
        getKPID(name);
        sleep(100);
    }
    PRINT_TEST_END
}

void testOpenPlus()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        float speed = 0.5f;
        if (name == "RA_A" || name == "SA_A")
        {
            speed = 0.25f;
        }

        for (int i = 0; i < 3; i++) {
            openPlus(name, speed);
            sleep(200);
        }
        for (int i = 0; i < 3; i++) {
            openPlus(name, -speed);
            sleep(200);
        }
        for (int i = 0; i < 5; i++) {
            openPlus(name, 0.0f);
            sleep(200);
        }
    }
    PRINT_TEST_END
}

void testOpenPlusWithAbs()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        float speed = 0.5f;
        if (name == "RA_A" || name == "SA_A")
        {
            speed = 0.25f;
        }
        if (name == "RA_C" || name == "SA_C" || name == "RA_D")
        {
            speed = 1.0f;
        }

        for (int i = 0; i < 6; i++) {
            openPlus(name, speed);
            sleep(200);
            absEnc(name);
            sleep(200);
        }
        for (int i = 0; i < 6; i++) {
            openPlus(name, 0.0f);
            sleep(200);
            absEnc(name);
            sleep(200);
        }
        for (int i = 0; i < 6; i++) {
            openPlus(name, -speed);
            sleep(200);
            absEnc(name);
            sleep(200);
        }
        for (int i = 0; i < 6; i++) {
            openPlus(name, 0.0f);
            sleep(200);
            absEnc(name);
            sleep(200);
        }
        std::cout << std::endl;
    }
    PRINT_TEST_END    
}

void allDevBoardFunctions(string name)
{
    PRINT_TEST_START
    printf("turning rib %s off", name.c_str());
    off(name);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("turning rib %s on", name.c_str());
    on(name);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("setting pwm max to 0.75");
    configPWM(name, 75);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("turning setting speed to 0.5");
    openPlus(name, 0.5);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    printf("turning setting speed to 0");
    openPlus(name, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("turning rib %s off", name.c_str());
    off(name);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    PRINT_TEST_END
}

void testAllDevBoardFunctions()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        allDevBoardFunctions(name);
    }
    PRINT_TEST_END
}

void testMax(string name)
{
    PRINT_TEST_START
    printf("setting max to 1.0");
    configPWM(name, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("setting speed to 0.5");
    openPlus(name, 50);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("setting max to 0.5");
    configPWM(name, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("trying to set speed to 0.75 (should not change)");
    openPlus( name, 75);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("setting speed to 0.0");
    openPlus(name, 0);
    // runs for five seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    PRINT_TEST_END
}

void testOnOff()
{
    for (auto name : motor_names)
    {
        // sets speed to half speed
        on(name);
        sleep(20);
        off(name);
        sleep(20);
    }
}

int main()
{
    motor_names.push_back("RA_A");
    motor_names.push_back("RA_B");
    motor_names.push_back("RA_C");
    motor_names.push_back("RA_D");
    motor_names.push_back("RA_E");
    motor_names.push_back("RA_F");

    printf("Initializing virtual controllers\n");
    ControllerMap::init();

    printf("Initializing I2C bus\n");
    I2C::init();

    testMakeLive();
    // testOn();
    // testConfigPWM();
    // testAdjust();

    while (1)
    {
        // testClosed();
	    // testQuadEnc();
        // testOpenPlusWithAbs();
        testOpenPlus();
        // testAbsEnc();
        sleep(100);

    }

    testOff();
    return 0;
}