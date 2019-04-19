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
#include "rover_msgs/WheelSpeeds.hpp"
#include "rover_msgs/TalonConfig.hpp"
#include "rover_msgs/SAMotors.hpp"
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
const double WHEEL_ENC_CPR = 1024.0;
const double ABS_ENC_CPR = 4096.0;
const double PI = 3.14159;

const int JOINT_A_OFFSET = -330;
const int JOINT_B_OFFSET = -1925;
const int JOINT_C_OFFSET = -569;
const int JOINT_D_OFFSET = -4448;
const int JOINT_E_OFFSET = -3729;

bool ARM_ENABLED = 0;
bool SA_ENABLED = 0;

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

mutex canLock;
deque<TalonSRX> talons; // Use deque because TalonSRX has no move constructor.

double encoderUnitsToRadians(int units, int cpr, int offset) {
    int x = units - offset;
    // Handle encoders modding on power cycle
    if(x > cpr / 2.0)
        x -= cpr;
    else if(x < -cpr / 2.0)
        x += cpr;
    return (x / static_cast<double>(cpr)) * 2 * PI;
}

int radiansToEncoderUnits(double angle, int cpr, int offset) {
    double x = angle / (2 * PI);
    return (x * cpr) + offset;
}

double encoderSpeedToRPS(int speed, int cpr) {
    double rotationsPerMs = static_cast<double>(speed) / 100;
    return (rotationsPerMs * 1000) / cpr;
}

bool isDriveMotor(int id) {
    return id < 4;
}

int jointIDtoTalonID(int id) {
    return id + 4;
}

class LCMHandlers {
public:
    // Drive mobility motors
    void drive(const lcm::ReceiveBuffer* receiveBuffer,
               const string& channel,
               const DriveMotors* msg) {
        lock_guard<mutex> scopedLock(canLock);

        talons[Talons::leftFront].Set(ControlMode::PercentOutput, msg->left);
        talons[Talons::rightFront].Set(ControlMode::PercentOutput, msg->right);
    }

    // Drive arm motor via open-loop throttle demands
    void armDrive(const lcm::ReceiveBuffer* receiveBuffer,
                   const string& channel,
                   const OpenLoopRAMotor* msg) {
        lock_guard<mutex> scopedLock(canLock);

        if(!ARM_ENABLED)
            return;
        
        talons[jointIDtoTalonID(msg->joint_id)].Set(ControlMode::PercentOutput, msg->speed);
    }

    // Drive arm motors via closed-loop position demands
    void armIKDrive(const lcm::ReceiveBuffer* receiveBuffer,
                    const string& channel,
                    const ArmPosition* msg) {
        lock_guard<mutex> scopedLock(canLock);

        if(!ARM_ENABLED)
            return;
        
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
    void configPID(const lcm::ReceiveBuffer* receiveBuffer,
                    const string& channel,
                    const PIDConstants* msg) {
        lock_guard<mutex> scopedLock(canLock);

        talons[msg->deviceID].Config_kP(0, msg->kP);
        talons[msg->deviceID].Config_kI(0, msg->kI);
        talons[msg->deviceID].Config_kD(0, msg->kD);
    }

    // Set output demand of an individual talon
    void setDemand(const lcm::ReceiveBuffer* receiveBuffer,
                    const string& channel,
                    const SetDemand* msg) {
        lock_guard<mutex> scopedLock(canLock);

        if(!isDriveMotor(msg->deviceID) && !ARM_ENABLED)
            return;

        ControlMode controlMode = static_cast<ControlMode>(msg->control_mode);
        talons[msg->deviceID].Set(controlMode, msg->value);
    }

    // Set SA/Arm talon configuration
    void talonConfig(const lcm::ReceiveBuffer* receiveBuffer,
                      const string& channel,
                      const TalonConfig* msg) {
        lock_guard<mutex> scopedLock(canLock);

        // Error case: Both configs requested on
        if(msg->enable_arm && msg->enable_sa) {
            cout << "Error: Arm and SA cannot be enabled together." << endl;
            return;
        }
        // Arm Configuration
        else if(!ARM_ENABLED && msg->enable_arm) {
            talons[Talons::saCarriage].EnableVoltageCompensation(false);
            talons[Talons::saFourBar].EnableVoltageCompensation(false);
            talons[Talons::saMicroX].EnableVoltageCompensation(false);
            talons[Talons::saMicroY].EnableVoltageCompensation(false);
            talons[Talons::saMicroZ].EnableVoltageCompensation(false);
            SA_ENABLED = 0;
            talons[Talons::armJointB].ConfigVoltageCompSaturation(12.0);
            talons[Talons::armJointC].ConfigVoltageCompSaturation(12.0);
            talons[Talons::armJointF].ConfigVoltageCompSaturation(5.0);
            talons[Talons::armJointG].ConfigVoltageCompSaturation(12.0);
            talons[Talons::armJointB].EnableVoltageCompensation(true);
            talons[Talons::armJointC].EnableVoltageCompensation(true);
            talons[Talons::armJointF].EnableVoltageCompensation(true);
            talons[Talons::armJointG].EnableVoltageCompensation(true);
            ARM_ENABLED = 1;
        } 
        // SA Configuration
        else if (!SA_ENABLED && msg->enable_sa) {
            talons[Talons::armJointB].EnableVoltageCompensation(false);
            talons[Talons::armJointC].EnableVoltageCompensation(false);
            talons[Talons::armJointF].EnableVoltageCompensation(false);
            talons[Talons::armJointG].EnableVoltageCompensation(false);
            ARM_ENABLED = 0;
            talons[Talons::saCarriage].ConfigVoltageCompSaturation(12.0);
            talons[Talons::saFourBar].ConfigVoltageCompSaturation(12.0);
            talons[Talons::saMicroX].ConfigVoltageCompSaturation(12.0);
            talons[Talons::saMicroY].ConfigVoltageCompSaturation(12.0);
            talons[Talons::saMicroZ].ConfigVoltageCompSaturation(12.0);
            talons[Talons::saCarriage].EnableVoltageCompensation(true);
            talons[Talons::saFourBar].EnableVoltageCompensation(true);
            talons[Talons::saMicroX].EnableVoltageCompensation(true);
            talons[Talons::saMicroY].EnableVoltageCompensation(true);
            talons[Talons::saMicroZ].EnableVoltageCompensation(true);
            SA_ENABLED = 1;
        }
    }

    // Drive SA Motors
    void saMotors(const lcm::ReceiveBuffer* receiveBuffer,
                   const string& channel,
                   const SAMotors* msg) {
        lock_guard<mutex> scopedLock(canLock);

        if(!SA_ENABLED)
            return;

        talons[Talons::saCarriage].Set(ControlMode::PercentOutput, msg->carriage);
        talons[Talons::saFourBar].Set(ControlMode::PercentOutput, msg->four_bar);
        talons[Talons::saDrillFront].Set(ControlMode::PercentOutput, msg->front_drill);
        talons[Talons::saDrillBack].Set(ControlMode::PercentOutput, msg->back_drill);
        talons[Talons::saMicroX].Set(ControlMode::PercentOutput, msg->micro_x);
        talons[Talons::saMicroY].Set(ControlMode::PercentOutput, msg->micro_y);
        talons[Talons::saMicroZ].Set(ControlMode::PercentOutput, msg->micro_z);
    }
};

/* Configuration Functions */
void configFollowerMode() {
    // Make rear drive motors follow output routines of front motors
    talons[Talons::leftBack].Follow(talons[Talons::leftFront]);
    talons[Talons::rightBack].Follow(talons[Talons::rightFront]);
}

void configPIDConstants() {
    talons[Talons::armJointA].Config_kP(0, 4.0);
    talons[Talons::armJointA].Config_kI(0, 0.0001);
    talons[Talons::armJointA].ConfigAllowableClosedloopError(0, 40);
    talons[Talons::armJointB].Config_kP(0, 2.0);
    talons[Talons::armJointB].Config_kI(0, 0.00002);
    talons[Talons::armJointC].Config_kP(0, 1.9);
    talons[Talons::armJointC].Config_kI(0, 0.00008);
    talons[Talons::armJointD].Config_kP(0, 1.5);
    talons[Talons::armJointD].Config_kI(0, 0.00002);
    talons[Talons::armJointE].Config_kP(0, 1.5);
    talons[Talons::armJointE].Config_kI(0, 0.00001);
}

void configCurrentLimits() {
    // TODO (not SAR-Critical)
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
    talons[Talons::armJointA].SetSensorPhase(false);
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

        WheelSpeeds wheel_msg;
        // Get mobility encoder velocities
        int lfEncVel = talons[Talons::leftFront].GetSelectedSensorVelocity();
        int lbEncVel = talons[Talons::leftBack].GetSelectedSensorVelocity();
        int rfEncVel = talons[Talons::rightFront].GetSelectedSensorVelocity();
        int rbEncVel = talons[Talons::rightBack].GetSelectedSensorVelocity();
        // Convert raw encoder speeds to RPS
        wheel_msg.left_front = encoderSpeedToRPS(lfEncVel, WHEEL_ENC_CPR);
        wheel_msg.left_back = encoderSpeedToRPS(lbEncVel, WHEEL_ENC_CPR);
        wheel_msg.right_front = encoderSpeedToRPS(rfEncVel, WHEEL_ENC_CPR);
        wheel_msg.right_back = encoderSpeedToRPS(rbEncVel, WHEEL_ENC_CPR);
        // Publish
        lcm.publish("/wheel_speeds", &wheel_msg);
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
    lcm.subscribe("/talon_config", &LCMHandlers::talonConfig, &lcmHandlers);
    lcm.subscribe("/sa_motors", &LCMHandlers::saMotors, &lcmHandlers);

    thread enableFrameThread(sendEnableFrames);
    thread encoderThread(publishEncoderData, ref(lcm));
    while (lcm.handle() == 0);

    return 0;
}
