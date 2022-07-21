#include "rover.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rover_msgs/TargetBearing.hpp"
#include "utilities.hpp"

Rover::Rover(const rapidjson::Document& config, lcm::LCM& lcmObject)
    : mConfig(config), mLcmObject(lcmObject),
      mTurningBearingPid(PidLoop(config["bearingPid"]["kP"].GetDouble(),
                                 config["bearingPid"]["kI"].GetDouble(),
                                 config["bearingPid"]["kD"].GetDouble())
                                 .withMaxInput(360.0)
                                 .withThreshold(mConfig["navThresholds"]["turningBearing"].GetDouble())),
      mDriveBearingPid(PidLoop(config["driveBearingPid"]["kP"].GetDouble(),
                               config["driveBearingPid"]["kI"].GetDouble(),
                               config["driveBearingPid"]["kD"].GetDouble())
                               .withMaxInput(360.0)
                               .withThreshold(mConfig["navThresholds"]["turningBearing"].GetDouble())),
      mLongMeterInMinutes(-1),
      mTurning(true),
      mDriving(false),
      mBackingUp(false),
      mNeedToSetTurnStart(true) {
}// Rover()

/***
 * Drive to the global position defined by destination, turning if necessary.
 *
 * @param destination   Global destination
 * @param stopDistance  If we are this distance or closer, stop
 * @param dt            Delta time in seconds
 * @return              Whether we have reached the target
 */
bool Rover::drive(const Odometry& destination, double stopDistance, double dt) {
    double distance = estimateDistance(mOdometry, destination);
    double bearing = estimateBearing(mOdometry, destination);
    return drive(distance, bearing, stopDistance, dt);
}// drive()

bool Rover::drive(double distance, double bearing, double threshold, double dt) {
    if (distance < threshold) {
        mTurning = true;
        mDriving = false;
        mBackingUp = false;
        mNeedToSetTurnStart = true;
        return true;
    }
    if (mNeedToSetTurnStart) {
        mTurnStartTime = NOW;
        std::cout << "staring turn clock" << std::endl;
        mNeedToSetTurnStart = false;
    }
    TargetBearing targetBearingLCM = {bearing};
    std::string const& targetBearingChannel = mConfig["lcmChannels"]["targetBearingChannel"].GetString();
    mLcmObject.publish(targetBearingChannel, &targetBearingLCM);
    auto straightTime = std::chrono::milliseconds(2000);
    auto totalTime = std::chrono::milliseconds(4000);
    auto turnTimeout = std::chrono::milliseconds(15000);
    if (mBackingUp) {
        std::cout << "backing out" << std::endl;
        if (NOW - mBackingUpStartTime > totalTime) {
            mBackingUp = false;
            mTurning = true;
            mDriving = false;
            mTurnStartTime = NOW;
            std::cout << "staring turn clock" << std::endl;
        } else if (NOW - mBackingUpStartTime > straightTime) {
            publishAutonDriveCmd(-1.0, -0.5);
        } else {
            publishAutonDriveCmd(-1.0, -1.0);
        }
    } else if (mTurning) {
        std::cout << "turning" << std::endl;
        if (turn(bearing, dt)) {
            mTurning = false;
            mDriving = true;
            mBackingUp = false;
        }
        if (!mBackingUp) {
            //std::cout << NOW - mTurnStartTime << std::endl;
            if (NOW - mTurnStartTime > turnTimeout) {
                mBackingUpStartTime = NOW;
                mBackingUp = true;
                mTurning = false;
                mDriving = false;
            }
        }
    } else {
        std::cout << "driving" << std::endl;
        double destinationBearing = mod(bearing, 360);
        double turningEffort = mDriveBearingPid.update(mOdometry.bearing_deg, destinationBearing, dt);
        if (mDriveBearingPid.error(mOdometry.bearing_deg, destinationBearing) > mConfig["navThresholds"]["drivingBearing"].GetDouble()) {
            mTurning = true;
            mTurnStartTime = NOW;
            std::cout << "starting turn clock" << std::endl;
            mDriving = false;
        }
        // When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        // if we need to turn clockwise, turning effort will be positive, so leftVel will be 1, and rightVel will be in between 0 and 1
        // if we need to turn ccw, turning effort will be negative, so rightVel will be 1 and leftVel will be in between 0 and 1
        // TODO: use std::clamp
        double leftVel = std::min(1.0, std::max(0.0, 1.0 + turningEffort));
        double rightVel = std::min(1.0, std::max(0.0, 1.0 - turningEffort));
        publishAutonDriveCmd(leftVel, rightVel);
    }

    return false;
}// drive()

bool Rover::driveBackwards(const Odometry& destination, double stopDistance, double dt) {
    double distance = estimateDistance(mOdometry, destination);
    double bearing = estimateBearing(mOdometry, destination);
    return driveBackwards(distance, bearing, stopDistance, dt);
}// drive()

bool Rover::driveBackwards(double distance, double bearing, double threshold, double dt) {
    if (distance < threshold) {
        mTurning = true;
        mDriving = false;
        mBackingUp = false;
        mNeedToSetTurnStart = true;
        return true;
    }
    if (mNeedToSetTurnStart) {
        mTurnStartTime = NOW;
        std::cout << "staring turn clock" << std::endl;
        mNeedToSetTurnStart = false;
    }

    bearing = mod(bearing+180,360);
    TargetBearing targetBearingLCM = {bearing};
    std::string const& targetBearingChannel = mConfig["lcmChannels"]["targetBearingChannel"].GetString();
    mLcmObject.publish(targetBearingChannel, &targetBearingLCM);
    auto straightTime = std::chrono::milliseconds(2000);
    auto totalTime = std::chrono::milliseconds(4000);
    auto turnTimeout = std::chrono::milliseconds(15000);
    if (mBackingUp) {
        std::cout << "backing out" << std::endl;
        if (NOW - mBackingUpStartTime > totalTime) {
            mBackingUp = false;
            mTurning = true;
            mDriving = false;
            mTurnStartTime = NOW;
            std::cout << "staring turn clock" << std::endl;
        } else if (NOW - mBackingUpStartTime > straightTime) {
            publishAutonDriveCmd(1.0, 0.5);
        } else {
            publishAutonDriveCmd(1.0, 1.0);
        }
    } else if (mTurning) {
        std::cout << "turning" << std::endl;
        if (turn(bearing, dt)) {
            mTurning = false;
            mDriving = true;
            mBackingUp = false;
        }
        if (!mBackingUp) {
            //std::cout << NOW - mTurnStartTime << std::endl;
            if (NOW - mTurnStartTime > turnTimeout) {
                mBackingUpStartTime = NOW;
                mBackingUp = true;
                mTurning = false;
                mDriving = false;
            }
        }
    } else {
        std::cout << "driving" << std::endl;
        double destinationBearing = mod(bearing, 360);
        double turningEffort = mDriveBearingPid.update(mOdometry.bearing_deg, destinationBearing, dt);
        if (mDriveBearingPid.error(mOdometry.bearing_deg, destinationBearing) > mConfig["navThresholds"]["drivingBearing"].GetDouble()) {
            mTurning = true;
            mTurnStartTime = NOW;
            std::cout << "starting turn clock" << std::endl;
            mDriving = false;
        }
        // When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        // if we need to turn clockwise, turning effort will be positive, so leftVel will be 1, and rightVel will be in between 0 and 1
        // if we need to turn ccw, turning effort will be negative, so rightVel will be 1 and leftVel will be in between 0 and 1
        // TODO: use std::clamp
        double leftVel = std::min(1.0, std::max(0.0, 1.0 - turningEffort));
        double rightVel = std::min(1.0, std::max(0.0, 1.0 + turningEffort));
        publishAutonDriveCmd(-leftVel, -rightVel);
    }

    return false;
}// drive()

/** @brief Turn to a global point */
bool Rover::turn(Odometry const& destination, double dt) {
    double bearing = estimateBearing(mOdometry, destination);
    return turn(bearing, dt);
}// turn()

/** @brief Turn to an absolute bearing */
bool Rover::turn(double absoluteBearing, double dt) {
    if (mTurningBearingPid.isOnTarget(mOdometry.bearing_deg, absoluteBearing)) {
        return true;
    }
    double turningEffort = mTurningBearingPid.update(mOdometry.bearing_deg, absoluteBearing, dt);
    // To turn in place we apply +turningEffort, -turningEffort on either side and make sure they're both within [-1, 1]
    double leftVel = std::max(std::min(1.0, +turningEffort), -1.0);
    double rightVel = std::max(std::min(1.0, -turningEffort), -1.0);
    publishAutonDriveCmd(leftVel, rightVel);
    return false;
}// turn()

void Rover::stop() {
    mTurning = true;
    mBackingUp = false;
    mDriving = false;
    mNeedToSetTurnStart = true;
    publishAutonDriveCmd(0.0, 0.0);
}// stop()

/** @brief Calculates the conversion from minutes to meters based on the rover's current latitude */
double Rover::longMeterInMinutes() const {
    if (mLongMeterInMinutes <= 0.0) {
        throw std::runtime_error("Invalid conversion");
    }
    return mLongMeterInMinutes;
}// longMeterInMinutes()

/** @return Rover's turning PID controller */
PidLoop& Rover::turningBearingPid() {
    return mTurningBearingPid;
}// turningBearingPid()

// Gets the rover's turning pid while driving object
PidLoop& Rover::drivingBearingPid() {
    return mDriveBearingPid;
}// drivingBearingPid()

void Rover::publishAutonDriveCmd(const double leftVel, const double rightVel) {
    AutonDriveControl driveControl{
            .left_percent_velocity = leftVel,
            .right_percent_velocity = rightVel};
    std::string autonDriveControlChannel = mConfig["lcmChannels"]["autonDriveControlChannel"].GetString();
    mLcmObject.publish(autonDriveControlChannel, &driveControl);
}

NavState const& Rover::currentState() const {
    return mCurrentState;
}// currentState()

Enable const& Rover::autonState() const {
    return mAutonState;
}// autonState()

Odometry const& Rover::odometry() const {
    return mOdometry;
}// odometry()

void Rover::setAutonState(Enable state) {
    mAutonState = state;
}// setAutonState()

void Rover::setOdometry(const Odometry& odometry) {
    mOdometry = odometry;
}// setOdometry()

void Rover::setState(NavState state) {
    mCurrentState = state;
}// setState()

void Rover::setLongMeterInMinutes(double l) {
    mLongMeterInMinutes = l;
}//setLongMeterInMinutes()
