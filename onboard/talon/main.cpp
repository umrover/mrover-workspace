#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "rover_msgs/DriveMotors.hpp"
#include "rover_msgs/OpenLoopRAMotor.hpp"
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/PIDConstants.hpp"
#include "rover_msgs/SetDemand.hpp"
#include "rover_msgs/Encoder.hpp"
#include <string>
#include <deque>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <lcm/lcm-cpp.hpp>

using namespace std;
using namespace rover_msgs;

const string INTERFACE = "can0";
const int NUM_TALONS = 11;
const double ABS_ENC_CPR = 4096.0;
const double PI = 3.14159;

const int JOINT_A_OFFSET = 0; // TODO
const int JOINT_B_OFFSET = -2100;
const int JOINT_C_OFFSET = -3000;
const int JOINT_D_OFFSET = -824;
const int JOINT_E_OFFSET = -753;

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
    armJointG = 10
};

mutex canLock;
deque<TalonSRX> talons; // Use deque because TalonSRX has no move constructor.

double encoderUnitsToRadians(int units, int cpr, int offset) {
    int x = units - offset;
    return (x / static_cast<double>(cpr)) * 2 * PI;
}

int radiansToEncoderUnits(double angle, int cpr, int offset) {
    double x = angle / (2 * PI);
    return (x * cpr) + offset;
}

class LCMHandlers {
public:
    // Drive mobility motors
    void drive(const lcm::ReceiveBuffer* recieveBuffer,
               const string& channel,
               const DriveMotors* msg) {
        lock_guard<mutex> scopedLock(canLock);
        talons[Talons::leftFront].Set(ControlMode::PercentOutput, msg->left);
        talons[Talons::rightFront].Set(ControlMode::PercentOutput, msg->right);
    }

    // Drive arm motor via open-loop throttle demands
    void armDrive(const lcm::ReceiveBuffer* recieveBuffer,
                   const string& channel,
                   const OpenLoopRAMotor* msg) {
        lock_guard<mutex> scopedLock(canLock);
        talons[msg->joint_id + 4].Set(ControlMode::PercentOutput, msg->speed);
    }

    // Drive arm motors via closed-loop position demands
    void armIKDrive(const lcm::ReceiveBuffer* recieveBuffer,
                    const string& channel,
                    const ArmPosition* msg) {
        lock_guard<mutex> scopedLock(canLock);
        
        int aUnits = radiansToEncoderUnits(msg->joint_a, ABS_ENC_CPR, JOINT_A_OFFSET);
        int bUnits = radiansToEncoderUnits(msg->joint_b, 2* ABS_ENC_CPR, JOINT_B_OFFSET);
        int cUnits = radiansToEncoderUnits(msg->joint_c, ABS_ENC_CPR, JOINT_C_OFFSET);
        int dUnits = radiansToEncoderUnits(msg->joint_d, ABS_ENC_CPR, JOINT_D_OFFSET);
        int eUnits = radiansToEncoderUnits(msg->joint_e, ABS_ENC_CPR, JOINT_E_OFFSET);

        talons[Talons::armJointA].Set(ControlMode::Position, aUnits);
        talons[Talons::armJointB].Set(ControlMode::Position, bUnits);
        talons[Talons::armJointC].Set(ControlMode::Position, cUnits);
        talons[Talons::armJointD].Set(ControlMode::Position, dUnits);
        talons[Talons::armJointE].Set(ControlMode::Position, eUnits);
    }

    // Configure a talon's PID constants
    void configPID(const lcm::ReceiveBuffer* recieveBuffer,
                    const string& channel,
                    const PIDConstants* msg) {
        lock_guard<mutex> scopedLock(canLock);
        talons[msg->deviceID].Config_kP(0, msg->kP);
        talons[msg->deviceID].Config_kI(0, msg->kI);
        talons[msg->deviceID].Config_kD(0, msg->kD);
    }

    // Set output demand of an individual talon
    void setDemand(const lcm::ReceiveBuffer* recieveBuffer,
                    const string& channel,
                    const SetDemand* msg) {
        lock_guard<mutex> scopedLock(canLock);
        ControlMode controlMode = static_cast<ControlMode>(msg->control_mode);
        talons[msg->deviceID].Set(controlMode, msg->value);
    }
};

/* Configuration Functions */
void configFollowerMode() {
    // Make rear drive motors follow output routines of front motors
    talons[Talons::leftBack].Follow(talons[Talons::leftFront]);
    talons[Talons::rightBack].Follow(talons[Talons::rightFront]);
}

void configPIDConstants() {
    // TODO: Configure Joint A
    talons[Talons::armJointB].Config_kP(0, 2.0);
    talons[Talons::armJointB].Config_kI(0, 0.0005);
    talons[Talons::armJointC].Config_kP(0, 2.7);
    talons[Talons::armJointC].Config_kI(0, 0.0005);
    talons[Talons::armJointD].Config_kP(0, 1.7);
    talons[Talons::armJointD].Config_kI(0, 0.0002);
    talons[Talons::armJointE].Config_kP(0, 2.2);
    talons[Talons::armJointE].Config_kI(0, 0.0005);
}

void configCurrentLimits() {
    // TODO (not SAR-Critical)
}

void configVoltageCompensation() {
    // Arm Joints B,C,F,G have 12V motors
    talons[Talons::armJointB].ConfigVoltageCompSaturation(12.0);
    talons[Talons::armJointC].ConfigVoltageCompSaturation(12.0);
    talons[Talons::armJointF].ConfigVoltageCompSaturation(12.0);
    talons[Talons::armJointG].ConfigVoltageCompSaturation(12.0);
    talons[Talons::armJointB].EnableVoltageCompensation(true);
    talons[Talons::armJointC].EnableVoltageCompensation(true);
    talons[Talons::armJointF].EnableVoltageCompensation(true);
    talons[Talons::armJointG].EnableVoltageCompensation(true);
}

void configFeedbackDevices() {
    // Drive Motors: Quadrature Encoders
    talons[Talons::leftFront].ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder);
    talons[Talons::leftBack].ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder);
    talons[Talons::rightFront].ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder);
    talons[Talons::rightBack].ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder);
    // Arm Joints A-E: CTRE Absolute Encoders
    talons[Talons::armJointA].ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Absolute);
    // TODO: Check Joint A sensor phase
    talons[Talons::armJointB].ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Absolute);
    talons[Talons::armJointB].SetSensorPhase(true);
    talons[Talons::armJointC].ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Absolute);
    talons[Talons::armJointC].SetSensorPhase(true);
    talons[Talons::armJointD].ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Absolute);
    talons[Talons::armJointD].SetSensorPhase(true);
    talons[Talons::armJointE].ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Absolute);
    talons[Talons::armJointE].SetSensorPhase(true);
    // Arm Joints F-G: CTRE Relative Encoders
    talons[Talons::armJointF].ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder);
    talons[Talons::armJointG].ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder);
}

void configLimitSwitches() {
    // TODO (not SAR-Critical)
}

void configTalons() {
    configFollowerMode();
    configPIDConstants();
    configCurrentLimits();
    configVoltageCompensation();
    configFeedbackDevices();
    configLimitSwitches();
}

/* Periodic Functions */
void sendEnableFrames() {
    while (true) {
        this_thread::sleep_for(chrono::milliseconds(500));
        lock_guard<mutex> scopedLock(canLock);
        ctre::phoenix::unmanaged::FeedEnable(600);
    }
}

void publishEncoderData(lcm::LCM &lcm) {
    while (true) {
        this_thread::sleep_for(chrono::milliseconds(100));
        lock_guard<mutex> scopedLock(canLock);

        ArmPosition arm_msg;
        Encoder enc_msg;
        // Get raw arm encoder positions
        int aPosRaw = talons[Talons::armJointA].GetSelectedSensorPosition();
        int bPosRaw = talons[Talons::armJointB].GetSelectedSensorPosition();
        int cPosRaw = talons[Talons::armJointC].GetSelectedSensorPosition();
        int dPosRaw = talons[Talons::armJointD].GetSelectedSensorPosition();
        int ePosRaw = talons[Talons::armJointE].GetSelectedSensorPosition();
        enc_msg.joint_a = aPosRaw;
        enc_msg.joint_b = bPosRaw;
        enc_msg.joint_c = cPosRaw;
        enc_msg.joint_d = dPosRaw;
        enc_msg.joint_e = ePosRaw;
        enc_msg.joint_f = 0;
        // Convert raw positions to floating point rotations (Joint B at 2:1 gear ratio)
        arm_msg.joint_a = encoderUnitsToRadians(aPosRaw, ABS_ENC_CPR, JOINT_A_OFFSET);
        arm_msg.joint_b = encoderUnitsToRadians(bPosRaw, 2* ABS_ENC_CPR, JOINT_B_OFFSET);
        arm_msg.joint_c = encoderUnitsToRadians(cPosRaw, ABS_ENC_CPR, JOINT_C_OFFSET);
        arm_msg.joint_d = encoderUnitsToRadians(dPosRaw, ABS_ENC_CPR, JOINT_D_OFFSET);
        arm_msg.joint_e = encoderUnitsToRadians(ePosRaw, ABS_ENC_CPR, JOINT_E_OFFSET);
        // Publish
        lcm.publish("/arm_position", &arm_msg);
        lcm.publish("/encoder", &enc_msg);

        // Get mobility encoder velocities
        int lfEncVel = talons[Talons::leftFront].GetSelectedSensorVelocity();
        int lbEncVel = talons[Talons::leftBack].GetSelectedSensorVelocity();
        int rfEncVel = talons[Talons::rightFront].GetSelectedSensorVelocity();
        int rbEncVel = talons[Talons::rightBack].GetSelectedSensorVelocity();
        
        
        // TODO: Publish LCM Messages
    }
}

int main() {
    lcm::LCM lcm;
    if(!lcm.good()) {
        cout << "Error: Could not create LCM." << endl;
        return 1;
    }

    ctre::phoenix::platform::can::SetCANInterface(INTERFACE.c_str());

    // Instantiate and configure talons
    for(int i = 0; i < NUM_TALONS; ++i)
        talons.emplace_back(i);
    configTalons();

    LCMHandlers lcmHandlers;
    lcm.subscribe("/motor", &LCMHandlers::drive, &lcmHandlers);
    lcm.subscribe("/config_pid", &LCMHandlers::configPID, &lcmHandlers);
    lcm.subscribe("/set_demand", &LCMHandlers::setDemand, &lcmHandlers);
    lcm.subscribe("/arm_motors", &LCMHandlers::armDrive, &lcmHandlers);
    lcm.subscribe("/ik_ra_control", &LCMHandlers::armIKDrive, &lcmHandlers);

    thread enableFrameThread(sendEnableFrames);
    thread encoderThread(publishEncoderData, ref(lcm));
    while (lcm.handle() == 0);

    return 0;
}