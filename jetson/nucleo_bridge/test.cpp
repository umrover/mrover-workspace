#include <iostream>
#include "ControllerMap.h"
#include "Controller.h"
#include "Hardware.h"
#include "I2C.h"

#include <cmath> // M_PI
#include <thread>  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
#include <vector>

#define PRINT_TEST_START printf("Running Test #%2d, %s\n", ++num_tests_ran, __FUNCTION__);
#define PRINT_TEST_END printf("Finished Test #%2d, %s\n\n", num_tests_ran, __FUNCTION__);

#define ANGLE_ERROR_DEGREES 5

int num_tests_ran = 0;

std::vector<std::string> motor_names;

void sleep(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void off(std::string name)
{
    try
    {
        I2C::transact(ControllerMap::get_i2c_address(name), OFF, nullptr, nullptr);
        printf("test off successful on %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! test off failed on %s \n", name.c_str());
    }
}

void on(std::string name)
{
    try
    {
        I2C::transact(ControllerMap::get_i2c_address(name), ON, nullptr, nullptr);
        printf("test on successful on %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! test on failed on %s \n", name.c_str());
    }
}

float openPlus(std::string name, float speed)
{
    try
    {
        ControllerMap::controllers[name]->open_loop(speed);
        float rad_angle = ControllerMap::controllers[name]->current_angle;
        float deg_angle = rad_angle / (2 * M_PI) * 360;
        printf("openPlus %s: Quad degrees is %f \n", name.c_str(), deg_angle);
        return deg_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! openPlus failed on %s \n", name.c_str());
        return 0;
    }
}

float closedPlus(std::string name, float angle)
{
    try
    {
        ControllerMap::controllers[name]->closed_loop(0, angle);
        float rad_angle = ControllerMap::controllers[name]->current_angle;
        float deg_angle = rad_angle / (2 * M_PI) * 360;
        printf("closedPlus %s: Quad degrees is %f \n", name.c_str(), deg_angle);
        return deg_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! closed plus failed on %s \n", name.c_str());
        return 0;
    }
}

void configPWM(std::string name, int max_speed)
{
    try
    {
        uint8_t buffer[2];
        memcpy(buffer, UINT8_POINTER_T(&(max_speed)), sizeof(max_speed));
        I2C::transact(ControllerMap::get_i2c_address(name), CONFIG_PWM, buffer, nullptr);
        printf("config pwm transaction successful on %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! test config pwm failed on %s \n", name.c_str());
    }
}

void setKPID(std::string name, float p, float i, float d)
{
    try
    {
        ControllerMap::controllers[name]->config(p, i, d);
        printf("set kpid transaction successful on %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! test set kpid failed on %s \n", name.c_str());
    }
}

void getKPID(std::string name)
{
    try
    {
        uint8_t buffer[12];
        I2C::transact(ControllerMap::get_i2c_address(name), GET_K, nullptr, buffer);
        int p, i, d;
        memcpy(UINT8_POINTER_T(&p), buffer + 0, sizeof(p));
        memcpy(UINT8_POINTER_T(&i), buffer + 4, sizeof(i));
        memcpy(UINT8_POINTER_T(&d), buffer + 8, sizeof(d));
        printf("get kpid transaction successful on %s \n", name.c_str());
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! get kpid failed on %s \n", name.c_str());
    }
}

float quadEnc(std::string name) 
{
    try
    {
        ControllerMap::controllers[name]->quad_angle();
        float rad_angle = ControllerMap::controllers[name]->current_angle;
        float deg_angle = rad_angle / (2 * M_PI) * 360;
        printf("quadEnc %s: Quad in degrees: %f \n", name.c_str(), deg_angle);
        return deg_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE: quad transaction failed on %s \n", name.c_str());
        return 0;
    }
}

//uint16_t absEnc(int addr)
float absEnc(std::string name)
{
    try
    {
        float rad_angle = 0;
        I2C::transact(ControllerMap::get_i2c_address(name), ABS_ENC, nullptr, UINT8_POINTER_T(&rad_angle));
        float deg_angle = rad_angle / (2 * M_PI) * 360;
        printf("test abs %s: Absolute degrees is %f \n", name.c_str(), deg_angle);
        return deg_angle;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! abs failed on %s \n", name.c_str());
        return 0;
    }
}

void adjust(std::string name)
{
    try
    {
        // gets initial angles
        float quad_angle = quadEnc(name);
        float abs_angle = absEnc(name);

        if (name == "RA_F")
        {
            abs_angle = M_PI;
        }

        printf("BEFORE adjust %s: Quadrature is %f, absolute is %f, difference is %f \n", name.c_str(), quad_angle, abs_angle, quad_angle - abs_angle); 

        int32_t adjusted_quad = (abs_angle / (2 * M_PI)) * ControllerMap::controllers[name]->quad_cpr;;

        // adjust transaction 
        uint8_t buffer[4];
        memcpy(buffer, UINT8_POINTER_T(&(adjusted_quad)), sizeof(adjusted_quad));
        I2C::transact(ControllerMap::get_i2c_address(name), ADJUST, buffer, nullptr);
        //ControllerMap::controllers[name]->zero();

        // checks values after
        quad_angle = quadEnc(name);
        abs_angle = absEnc(name);

        if (name == "RA_F")
        {
            abs_angle = M_PI;
        }

        printf("AFTER adjust %s: Quadrature is %f, absolute is %f, difference is %f \n", name.c_str(), quad_angle, abs_angle, quad_angle - abs_angle); 

        if (std::abs(quad_angle - abs_angle) > .2)
        {
            printf("FAILED! test adjust transaction numerically failed on %s \n\n", name.c_str());
        }
        else
        {
            printf("FAILED! test adjust transaction successful on %s \n\n", name.c_str());
        }
    }
    catch (IOFailure &e)
    {
        printf("FAILED! test adjust transaction failed on %s \n\n", name.c_str());
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

void testQuadEnc()
{
    for (auto name : motor_names)
    {
        // test off
        float deg_angle = quadEnc(name);
        printf("[%s] Quad degrees: %i \n", name.c_str(), deg_angle);
        sleep(50;)
    }
}

void testAbsEnc()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        // test off
        
        float deg_angle = absEnc(name);
        printf("[%s] Absolute degrees: %f \n", name.c_str(), deg_angle);
        sleep(10);
    }
    PRINT_TEST_END
}

void testClosed()
{
    PRINT_TEST_START
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
                angle = quadEnc(name);

                target = angle + offset[i];

                do 
                {
                    angle = closedPlus(name, target);
                    sleep(20);
                }

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
        float speed = 1.0f;

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

    float quad = 0;
    float abs = 0;

    for (auto name : motor_names)
    {
        float speed = 1.0f;

        for (int i = 0; i < 3; i++) {
            quad = openPlus(name, speed);
            sleep(200);
            abs = absEnc(name);
            sleep(200);
            float difference = quad - abs;
            if (abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad, abs, difference);
            }
        }
        for (int i = 0; i < 3; i++) {
            quad = openPlus(name, 0.0f);
            sleep(200);
            abs = absEnc(name);
            sleep(200);
            if (abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad, abs, difference);
            }
        }
        for (int i = 0; i < 3; i++) {
            quad = openPlus(name, -speed);
            sleep(200);
            abs = absEnc(name);
            sleep(200);
            if (abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad, abs, difference);
            }
        }
        for (int i = 0; i < 6; i++) {
            quad = openPlus(name, 0.0f);
            sleep(200);
            abs = absEnc(name);
            sleep(200);
            if (abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad, abs, difference);
            }
        }
        std::cout << std::endl;
    }
    PRINT_TEST_END    
}

void allDevBoardFunctions(std::string name)
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

void testMax(std::string name)
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

    // testOn();
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