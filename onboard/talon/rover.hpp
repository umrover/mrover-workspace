#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

// LCM Message Types
#include "rover_msgs/DriveMotors.hpp"
#include "rover_msgs/OpenLoopRAMotor.hpp"
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/SAMotors.hpp"
#include "rover_msgs/PIDConstants.hpp"
#include "rover_msgs/SetDemand.hpp"
#include "rover_msgs/TalonConfig.hpp"
#include "rover_msgs/Encoder.hpp"
#include "rover_msgs/WheelSpeeds.hpp"
#include "rover_msgs/AutonState.hpp"

#include <string>
#include <deque>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <lcm/lcm-cpp.hpp>

using namespace std;
using namespace rover_msgs;

const double PI = 3.14159;

enum Talons {
    leftFront = 0,
    leftBack = 1,
    rightFront = 2,
    rightBack = 3,
    armJointA = 4,
    armJointB = 5,
    armJointC = 6,
    armJointD = 7,
    armJointE = 8,
    armJointF = 9,
    armJointG = 10,
    saCarriage = 4,
    saFourBar = 5,
    saDrillFront = 6,
    saDrillBack = 7,
    saMicroX = 8,
    saMicroY = 9,
    saMicroZ = 10
};

class Rover {
private:
    deque<TalonSRX> talons;
    vector<int> offsets;
    vector<int> encs;
    vector<double> angles;
    vector<double> posfeeds;
    vector<double> negfeeds;
    bool armEnabled;
    bool saEnabled;
    bool autonomous;
    int wheelCPR;
    int armCPR;
    mutex canLock;

public:
    // Instantiates and configures rover's Talon SRX motor controllers.
    Rover(int numTalons, int _wheelCPR, int _armCPR);

    // Reads and updates robotic arm's current encoder counts
    // and joint angles. Also publishes to LCM.
    void publishEncoderData(lcm::LCM &lcm);

    // Enable talons for a given time
    void enable(int ms);
 
    /* LCM Message Handlers */

    // Drive mobility 
    void drive(const lcm::ReceiveBuffer* receiveBuffer, 
                const string& channel, const DriveMotors* msg);

    // Drive robotic arm (open-loop control).
    void armDrive(const lcm::ReceiveBuffer* receiveBuffer, 
                   const string& channel, const OpenLoopRAMotor* msg);

    // Drive robotic arm (IK control).
    void armIKDrive(const lcm::ReceiveBuffer* receiveBuffer,
                     const string& channel, const ArmPosition* msg);

    // Drive SA Motors
    void saMotors(const lcm::ReceiveBuffer* receiveBuffer,
                   const string& channel, const SAMotors* msg);

    // Config PID constants for a talon.
    void configPID(const lcm::ReceiveBuffer* receiveBuffer,
                    const string& channel, const PIDConstants* msg);

    // Set output routine for a talon.
    void setDemand(const lcm::ReceiveBuffer* receiveBuffer,
                    const string& channel, const SetDemand* msg);

    // Set talon configuration for arm or sa control.
    void talonConfig(const lcm::ReceiveBuffer* receiveBuffer,
                      const string& channel, const TalonConfig* msg);

    // Configure throttle ramping based on autonomy mode.
    void autonState(const lcm::ReceiveBuffer* receiveBuffer,
                      const string& channel, const AutonState* msg);

private:
    /* Configuration Functions */
    void configTalons();
    void configFollowerMode();
    void configBrakeMode();
    void configOpenLoopRamp();
    void configPIDConstants();
    void configCurrentLimits();
    void configFeedbackDevices();
    void configLimitSwitches();

    /* Helper Functions */
    bool isDriveMotor(int id) {
        return id < 4;
    }

    int jointIDtoTalonID(int id) {
        return id + 4;
    }

    // Converts an encoder count to an angle in the range [-PI, PI]
    double encoderUnitsToRadians(int units, int cpr, int offset) {
        int x = units - offset;
        double y = (x / static_cast<double>(cpr)) * 2 * PI;
        while (y > PI)
            y -= 2 * PI;
        while (y < -PI)
            y += 2 * PI;
        return y;
    }

    int deltaEncoderUnits(double angle, int cpr) {
        double x = angle / (2 * PI);
        return (x * cpr);
    }

    double encoderSpeedToRPS(int speed, int cpr) {
        double rotationsPerMs = static_cast<double>(speed) / 100;
        return (rotationsPerMs * 1000) / cpr;
    }
};