#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>

#include "rover.hpp"

using namespace std;
using namespace rover_msgs;

const string INTERFACE = "can0";
const int NUM_TALONS = 11;
const int WHEEL_ENC_CPR = 1024;
const int ARM_ENC_CPR = 4096;

void sendEnableFrames(Rover &rover) {
    while (true) {
        this_thread::sleep_for(chrono::milliseconds(500));
        rover.enable(600);
    }
}

void publishEncoderData(Rover &rover, lcm::LCM &lcm) {
    while (true) {
        this_thread::sleep_for(chrono::milliseconds(100));
        rover.publishEncoderData(lcm);
    }
}

int main() {
    lcm::LCM lcm;
    if(!lcm.good()) {
        cout << "Error: Could not create LCM." << endl;
        return 1;
    }

    ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());

    Rover rover(NUM_TALONS, WHEEL_ENC_CPR, ARM_ENC_CPR);

    lcm.subscribe("/motor", &Rover::drive, &rover);
    lcm.subscribe("/config_pid", &Rover::configPID, &rover);
    lcm.subscribe("/set_demand", &Rover::setDemand, &rover);
    lcm.subscribe("/arm_motors", &Rover::armDrive, &rover);
    lcm.subscribe("/ik_ra_control", &Rover::armIKDrive, &rover);
    lcm.subscribe("/talon_config", &Rover::talonConfig, &rover);
    lcm.subscribe("/sa_motors", &Rover::saMotors, &rover);
    lcm.subscribe("/auton", &Rover::autonState, &rover);

    thread enableFrameThread(sendEnableFrames, ref(rover));
    thread encoderThread(publishEncoderData, ref(rover), ref(lcm));
    while (lcm.handle() == 0);

    return 0;
}