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

// speed_1 is a float between -1 and 1
float openPlus(std::string name, float speed_1)
{
    try
    {
        ControllerMap::controllers[name]->open_loop(speed_1);
        float quad_angle_rad = ControllerMap::controllers[name]->get_current_angle();
        float quad_angle_deg = quad_angle_rad / (2 * M_PI) * 360;
        printf("openPlus %s: Quad degrees is %f \n", name.c_str(), quad_angle_deg);
        return quad_angle_deg;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! openPlus failed on %s \n", name.c_str());
        return 0;
    }
}

// target_angle_deg is targeted angle degrees
float closedPlus(std::string name, float target_angle_deg)
{
    try
    {
        float target_angle_rad = target_angle_deg * M_PI / 180.0;
        ControllerMap::controllers[name]->closed_loop(0, target_angle_rad);
        float quad_angle_rad = ControllerMap::controllers[name]->get_current_angle();
        float quad_angle_deg = quad_angle_rad / (2 * M_PI) * 360;
        float diff_deg = target_angle_deg - quad_angle_deg;
        printf("closedPlus %s: Target degrees is %f, quad degrees is %f, diff is %f\n", name.c_str(), target_angle_deg, quad_angle_deg, diff_deg);
        return quad_angle_deg;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! closed plus failed on %s \n", name.c_str());
        return 0;
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

// returns quad angle in degrees
float quadEnc(std::string name) 
{
    try
    {
        ControllerMap::controllers[name]->quad_angle();
        float quad_angle_rad = ControllerMap::controllers[name]->get_current_angle();
        float quad_angle_deg = quad_angle_rad / (2 * M_PI) * 360;
        printf("quadEnc %s: Quad in degrees: %f \n", name.c_str(), quad_angle_deg);
        return quad_angle_deg;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE: quad transaction failed on %s \n", name.c_str());
        return 0;
    }
}

// returns abs enc in degrees
float absEnc(std::string name)
{
    try
    {
        float abs_angle_rad = 0;
        I2C::transact(ControllerMap::get_i2c_address(name), ABS_ENC, nullptr, UINT8_POINTER_T(&abs_angle_rad));
        float abs_angle_deg = (abs_angle_rad - M_PI) / (2 * M_PI) * 360;
        printf("test abs %s: Absolute degrees is %f \n", name.c_str(), abs_angle_deg);
        return abs_angle_deg;
    }
    catch (IOFailure &e)
    {
        fprintf(stderr, "FAILURE! abs failed on %s \n", name.c_str());
        return 0;
    }
}

void testQuadEnc()
{
    for (auto name : motor_names)
    {
        float quad_angle_deg = quadEnc(name);
        printf("[%s] Quad degrees: %f \n", name.c_str(), quad_angle_deg);
        sleep(50);
    }
}

void testAbsEnc()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        float abs_angle_deg = absEnc(name);
        printf("[%s] Absolute degrees: %f \n", name.c_str(), abs_angle_deg);
        sleep(10);
    }
    PRINT_TEST_END
}

void testClosed()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        float p, i, d;
        p = ControllerMap::controllers[name]->kP;
        i = ControllerMap::controllers[name]->kI;
        d = ControllerMap::controllers[name]->kD;
        setKPID(name, p, i, d);  // highly optional, but useful as a sanity check
        printf("joint %s, kp is %f, ki is %f, kd is %f \n", name.c_str(), p, i, d);
        sleep(10);
    }

    while (1)
    {
        for (auto name : motor_names)
        {
            float current_angle_deg = 0;
            float offset_deg[4] = { 10, -10, 10, -10 };
            float target_deg = 0;

            for (int i = 0; i < 4; ++i) {

                printf("Attempting to head to position %i\n", i);
                current_angle_deg = quadEnc(name);

                target_deg = current_angle_deg + offset_deg[i];

                do 
                {
                    current_angle_deg = closedPlus(name, target_deg);
                    sleep(20);
                }
                while(std::abs(current_angle_deg - target_deg) > 0.6);

                printf("Arrived at position %i\n", i);
                sleep(400);
            }
        }
    }
    PRINT_TEST_END
}

void testOpenPlus()
{
    PRINT_TEST_START
    for (auto name : motor_names)
    {
        float speed_1 = 1.0f;

        for (int i = 0; i < 3; i++) {
            openPlus(name, speed_1);
            sleep(200);
        }
        for (int i = 0; i < 3; i++) {
            openPlus(name, -speed_1);
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

    float quad_angle_deg = 0.0f;
    float abs_angle_deg = 0.0f;

    for (auto name : motor_names)
    {
        float speed_1 = 1.0f;

        for (int i = 0; i < 3; i++) {
            quad_angle_deg = openPlus(name, speed_1);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(200);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        for (int i = 0; i < 3; i++) {
            quad_angle_deg = openPlus(name, 0.0f);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(200);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        for (int i = 0; i < 3; i++) {
            quad_angle_deg = openPlus(name, -speed_1);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(200);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        for (int i = 0; i < 6; i++) {
            quad_angle_deg = openPlus(name, 0.0f);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(200);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        std::cout << std::endl;
    }
    PRINT_TEST_END    
}

void testOpenPlusWithAbsWithDelays()
{
    PRINT_TEST_START

    float quad_angle_deg = 0.0f;
    float abs_angle_deg = 0.0f;

    for (auto name : motor_names)
    {
        float speed_1 = 1.0f;

        printf("Moving %s. \n\n", name.c_str());
        for (int i = 0; i < 3; i++) {
            quad_angle_deg = openPlus(name, speed_1);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(200);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        printf("Stopping %s. \n\n", name.c_str());
        for (int i = 0; i < 10; i++) {
            quad_angle_deg = openPlus(name, 0.0f);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(400);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        printf("Moving %s. \n\n", name.c_str());
        for (int i = 0; i < 3; i++) {
            quad_angle_deg = openPlus(name, -speed_1);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(200);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        printf("Stopping %s. \n\n", name.c_str());
        for (int i = 0; i < 10; i++) {
            quad_angle_deg = openPlus(name, 0.0f);
            sleep(200);
            abs_angle_deg = absEnc(name);
            sleep(400);
            float difference = quad_angle_deg - abs_angle_deg;
            if (std::abs(difference) >= ANGLE_ERROR_DEGREES) 
            {
                printf("ANGLE ERROR on %s! Quad is %f, absolute is %f, diff is %f \n\n", name.c_str(), quad_angle_deg, abs_angle_deg, difference);
            }
        }
        std::cout << std::endl;
    }
    PRINT_TEST_END    
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

    while (1)
    {
        testClosed();
	    // testQuadEnc();
        // testOpenPlusWithAbs();
        //testOpenPlusWithAbsWithDelays();
        // testOpenPlus();
        // testAbsEnc();
        sleep(100);

    }

    return 0;
}
