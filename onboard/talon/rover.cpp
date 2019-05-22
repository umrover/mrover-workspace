#include "rover.hpp"

using namespace std;
using namespace rover_msgs;

// Instantiates and configures rover's Talon SRX motor controllers.
Rover::Rover(int numTalons, int _wheelCPR, int _armCPR) : armEnabled(false), 
    saEnabled(false), autonomous(false), wheelCPR(_wheelCPR), armCPR(_armCPR) {
    // Offsets for arm joints A-E, feed forward constants
    offsets = {820, -2672, -1936, -769, 407};
    posfeeds = {0.1, 0.18, 0.18, 0.05, 0.07};
    negfeeds = {-0.1, -0.13, -0.09, -0.04, -0.03};

    // Initialize current encoder counts, joint angles
    for(int i = 0; i < 5; ++i) {
        encs.push_back(0);
        angles.push_back(0);
    }

    // Instantiate talons
    for(int i = 0; i < numTalons; ++i)
        talons.emplace_back(i);

    configTalons();
}

// Reads and updates robotic arm's current encoder counts
// and joint angles. Also publishes to LCM.
void Rover::publishEncoderData(lcm::LCM &lcm) {
    lock_guard<mutex> scopedLock(canLock);

    ArmPosition arm_msg;
    Encoder enc_msg;
    WheelSpeeds wheel_msg;
    
    // Get raw arm encoder positions
    int aPosRaw = talons[Talons::armJointA].GetSelectedSensorPosition();
    int bPosRaw = talons[Talons::armJointB].GetSelectedSensorPosition();
    int cPosRaw = talons[Talons::armJointC].GetSelectedSensorPosition();
    int dPosRaw = talons[Talons::armJointD].GetSelectedSensorPosition();
    int ePosRaw = talons[Talons::armJointE].GetSelectedSensorPosition();

    enc_msg.joint_a = encs[0] = aPosRaw;
    enc_msg.joint_b = encs[1] = bPosRaw;
    enc_msg.joint_c = encs[2] = cPosRaw;
    enc_msg.joint_d = encs[3] = dPosRaw;
    enc_msg.joint_e = encs[4] = ePosRaw;
    
    arm_msg.joint_a = angles[0] = encoderUnitsToRadians(aPosRaw, armCPR, offsets[0]);
    arm_msg.joint_b = angles[1] = encoderUnitsToRadians(bPosRaw, 2*armCPR, offsets[1]);
    arm_msg.joint_c = angles[2] = encoderUnitsToRadians(cPosRaw, armCPR, offsets[2]);
    arm_msg.joint_d = angles[3] = encoderUnitsToRadians(dPosRaw, armCPR, offsets[3]);
    arm_msg.joint_e = angles[4] = encoderUnitsToRadians(ePosRaw, armCPR, offsets[4]);

    // Handle jumping for Joint B's encoder
    if(arm_msg.joint_b < -PI / 4) {
        arm_msg.joint_b += PI;
        angles[1] += PI;
    }
    if(arm_msg.joint_b > 3*PI / 4) {
        arm_msg.joint_b -= PI;
        angles[1] -= PI;
    }

    // Get mobility encoder velocities
    int lfEncVel = talons[Talons::leftFront].GetSelectedSensorVelocity();
    int lbEncVel = talons[Talons::leftBack].GetSelectedSensorVelocity();
    int rfEncVel = talons[Talons::rightFront].GetSelectedSensorVelocity();
    int rbEncVel = talons[Talons::rightBack].GetSelectedSensorVelocity();

    wheel_msg.left_front = encoderSpeedToRPS(lfEncVel, wheelCPR);
    wheel_msg.left_back = encoderSpeedToRPS(lbEncVel, wheelCPR);
    wheel_msg.right_front = encoderSpeedToRPS(rfEncVel, wheelCPR);
    wheel_msg.right_back = encoderSpeedToRPS(rbEncVel, wheelCPR);
    
    // Publish
    lcm.publish("/arm_position", &arm_msg);
    lcm.publish("/encoder", &enc_msg);
    lcm.publish("/wheel_speeds", &wheel_msg);
}

// Enable talons for a given time
void Rover::enable(int ms) {
    lock_guard<mutex> scopedLock(canLock);
    ctre::phoenix::unmanaged::FeedEnable(ms);
}

/* LCM Message Handlers */

// Drive mobility 
void Rover::drive(const lcm::ReceiveBuffer* receiveBuffer, 
                   const string& channel, const DriveMotors* msg) {
    lock_guard<mutex> scopedLock(canLock);

    talons[Talons::leftFront].Set(ControlMode::PercentOutput, msg->left);
    talons[Talons::rightFront].Set(ControlMode::PercentOutput, msg->right);
}

// Drive robotic arm (open-loop control).
void Rover::armDrive(const lcm::ReceiveBuffer* receiveBuffer, 
                      const string& channel, const OpenLoopRAMotor* msg) {
    lock_guard<mutex> scopedLock(canLock);

    if(!armEnabled)
        return;
       
    talons[jointIDtoTalonID(msg->joint_id)].Set(ControlMode::PercentOutput, msg->speed);
}

// Drive robotic arm (IK control).
void Rover::armIKDrive(const lcm::ReceiveBuffer* receiveBuffer,
                        const string& channel, const ArmPosition* msg) {
    lock_guard<mutex> scopedLock(canLock);

    if(!armEnabled)
        return;

    vector<int> cmds_delta = {
        deltaEncoderUnits(msg->joint_a - angles[0], armCPR),
        deltaEncoderUnits(msg->joint_b - angles[1], 2*armCPR),
        deltaEncoderUnits(msg->joint_c - angles[2], armCPR),
        deltaEncoderUnits(msg->joint_d - angles[3], armCPR),
        deltaEncoderUnits(msg->joint_e - angles[4], armCPR)
    };

    vector<double> feeds = posfeeds;
    for (int i = 0; i < 5; ++i) {
        if (abs(cmds_delta[i]) < 1)
            feeds[i] = 0;
        else if (cmds_delta[i] < 0)
            feeds[i] = negfeeds[i];
    }

    talons[Talons::armJointA].Set(ControlMode::Position, encs[0] + cmds_delta[0], DemandType::DemandType_ArbitraryFeedForward, feeds[0]);
    talons[Talons::armJointB].Set(ControlMode::Position, encs[1] + cmds_delta[1], DemandType::DemandType_ArbitraryFeedForward, feeds[1]);
    talons[Talons::armJointC].Set(ControlMode::Position, encs[2] + cmds_delta[2], DemandType::DemandType_ArbitraryFeedForward, feeds[2]);
    talons[Talons::armJointD].Set(ControlMode::Position, encs[3] + cmds_delta[3], DemandType::DemandType_ArbitraryFeedForward, feeds[3]);
    talons[Talons::armJointE].Set(ControlMode::Position, encs[4] + cmds_delta[4], DemandType::DemandType_ArbitraryFeedForward, feeds[4]);

}

// Drive SA Motors
void Rover::saMotors(const lcm::ReceiveBuffer* receiveBuffer,
                      const string& channel, const SAMotors* msg) {
    lock_guard<mutex> scopedLock(canLock);

    if(!saEnabled)
        return;

    talons[Talons::saCarriage].Set(ControlMode::PercentOutput, msg->carriage);
    talons[Talons::saFourBar].Set(ControlMode::PercentOutput, msg->four_bar);
    talons[Talons::saDrillFront].Set(ControlMode::PercentOutput, msg->front_drill);
    talons[Talons::saDrillBack].Set(ControlMode::PercentOutput, msg->back_drill);
    talons[Talons::saMicroX].Set(ControlMode::PercentOutput, msg->micro_x);
    talons[Talons::saMicroY].Set(ControlMode::PercentOutput, msg->micro_y);
    talons[Talons::saMicroZ].Set(ControlMode::PercentOutput, msg->micro_z);
}

// Config PID constants for a talon.
void Rover::configPID(const lcm::ReceiveBuffer* receiveBuffer,
                       const string& channel, const PIDConstants* msg) {
    lock_guard<mutex> scopedLock(canLock);

    talons[msg->deviceID].Config_kP(0, msg->kP);
    talons[msg->deviceID].Config_kI(0, msg->kI);
    talons[msg->deviceID].Config_kD(0, msg->kD);
}

// Set output routine for a talon.
void Rover::setDemand(const lcm::ReceiveBuffer* receiveBuffer,
                       const string& channel, const SetDemand* msg) {
    lock_guard<mutex> scopedLock(canLock);

    if(!isDriveMotor(msg->deviceID) && !armEnabled)
        return;
    
    ControlMode controlMode = static_cast<ControlMode>(msg->control_mode);
    talons[msg->deviceID].Set(controlMode, msg->value);
}

// Set talon configuration for arm or sa control.
void Rover::talonConfig(const lcm::ReceiveBuffer* receiveBuffer,
                         const string& channel, const TalonConfig* msg) {
    lock_guard<mutex> scopedLock(canLock);

    // Error case: Both configs requested on
    if(msg->enable_arm && msg->enable_sa) {
        cout << "Error: Arm and SA cannot be enabled together." << endl;
        return;
    }
    // Arm Configuration
    else if(!armEnabled && msg->enable_arm) {
        talons[Talons::saCarriage].EnableVoltageCompensation(false);
        talons[Talons::saFourBar].EnableVoltageCompensation(false);
        talons[Talons::saMicroX].EnableVoltageCompensation(false);
        talons[Talons::saMicroY].EnableVoltageCompensation(false);
        talons[Talons::saMicroZ].EnableVoltageCompensation(false);
        saEnabled = 0;
        talons[Talons::armJointB].ConfigVoltageCompSaturation(24.0);
        talons[Talons::armJointC].ConfigVoltageCompSaturation(12.0);
        talons[Talons::armJointF].ConfigVoltageCompSaturation(9.0);
        talons[Talons::armJointG].ConfigVoltageCompSaturation(24.0);
        talons[Talons::armJointB].EnableVoltageCompensation(true);
        talons[Talons::armJointC].EnableVoltageCompensation(true);
        talons[Talons::armJointF].EnableVoltageCompensation(true);
        talons[Talons::armJointG].EnableVoltageCompensation(true);
        armEnabled = 1;
    } 
    // SA Configuration
    else if (!saEnabled && msg->enable_sa) {
        talons[Talons::armJointB].EnableVoltageCompensation(false);
        talons[Talons::armJointC].EnableVoltageCompensation(false);
        talons[Talons::armJointF].EnableVoltageCompensation(false);
        talons[Talons::armJointG].EnableVoltageCompensation(false);
        armEnabled = 0;
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
        saEnabled = 1;
    }
}

// Configure throttle ramping based on autonomy mode.
void Rover::autonState(const lcm::ReceiveBuffer* receiveBuffer,
                        const string& channel, const AutonState* msg) {
    lock_guard<mutex> scopedLock(canLock);

    if (msg->is_auton && !autonomous) {
        talons[Talons::leftFront].ConfigOpenloopRamp(0.25);
        talons[Talons::rightFront].ConfigOpenloopRamp(0.25);
        autonomous = true;
    } else if (!msg->is_auton && autonomous) {
        talons[Talons::leftFront].ConfigOpenloopRamp(0.0);
        talons[Talons::rightFront].ConfigOpenloopRamp(0.0);
        autonomous = false;
    }
}

/* Configuration Functions */
void Rover::configTalons() {
    configFollowerMode();
    configBrakeMode();
    configOpenLoopRamp();
    configPIDConstants();
    configCurrentLimits();
    configFeedbackDevices();
    configLimitSwitches();
}

void Rover::configFollowerMode() {
    talons[Talons::leftBack].Follow(talons[Talons::leftFront]);
    talons[Talons::rightBack].Follow(talons[Talons::rightFront]);
}

void Rover::configBrakeMode() {
    for (TalonSRX &talon : talons) {
        talon.SetNeutralMode(NeutralMode::Brake);
    }
}

void Rover::configOpenLoopRamp() {
    talons[Talons::leftFront].ConfigOpenloopRamp(0.0);
    talons[Talons::rightFront].ConfigOpenloopRamp(0.0);
}

void Rover::configPIDConstants() {
    talons[Talons::armJointA].Config_kP(0, 4.0);
    talons[Talons::armJointA].Config_kI(0, 0.0001);
    talons[Talons::armJointA].ConfigAllowableClosedloopError(0, 5);
    talons[Talons::armJointB].Config_kP(0, 3.0);
    talons[Talons::armJointB].Config_kI(0, 0.00002);
    talons[Talons::armJointC].Config_kP(0, 4.0);
    talons[Talons::armJointC].Config_kI(0, 0.00008);
    talons[Talons::armJointD].Config_kP(0, 2.0);
    talons[Talons::armJointD].Config_kI(0, 0.00002);
    talons[Talons::armJointE].Config_kP(0, 2.0);
    talons[Talons::armJointE].Config_kI(0, 0.00001);
}

void Rover::configCurrentLimits() {
    talons[Talons::leftFront].ConfigContinuousCurrentLimit(10);
    talons[Talons::leftFront].ConfigPeakCurrentLimit(0);
    talons[Talons::leftBack].ConfigContinuousCurrentLimit(10);
    talons[Talons::leftBack].ConfigPeakCurrentLimit(0);
    talons[Talons::rightFront].ConfigContinuousCurrentLimit(10);
    talons[Talons::rightFront].ConfigPeakCurrentLimit(0);
    talons[Talons::rightBack].ConfigContinuousCurrentLimit(10);
    talons[Talons::rightBack].ConfigPeakCurrentLimit(0);
}

void Rover::configFeedbackDevices() {
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

void Rover::configLimitSwitches() {
    // TODO (not SAR-Critical)
}