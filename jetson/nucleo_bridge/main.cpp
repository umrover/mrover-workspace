#include <string>
#include <unordered_map>
#include <iostream>
#include <thread>
#include <chrono>

#include "lcm/lcm-cpp.hpp"
#include "Controller.h"
#include "Hardware.h"
#include "LCMHandler.h"
#include "I2C.h"

//Handles instantiation of Controller objects, FrontEnd, and BackEnd classes

//The outgoing function calls on the LCMHandler's handle_outgoing() function every millisecond
void outgoing()
{
    while (true)
    {
        LCMHandler::handle_outgoing();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

//The incoming function calls on the LCMHandler's handle_incoming() function continuously
void incoming()
{
    while (true)
    {
        LCMHandler::handle_incoming();
    }
}

int main()
{
    printf("Initializing virtual controllers\n");
    ControllerMap::init();

    printf("Initializing LCM bus\n");
    LCMHandler::init();

    printf("Initializing I2C bus\n");
    I2C::init();

    printf("Initialization Done. Looping. Reduced output for program speed.\n");
    std::thread outThread(&outgoing);
    std::thread inThread(&incoming);

    outThread.join();
    inThread.join();

    return 0;
}
