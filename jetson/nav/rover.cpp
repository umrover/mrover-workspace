#include "rover.hpp"
#include "utilities.hpp"
#include "rover_msgs/TargetBearing.hpp"

#include <cmath>
#include <iostream>

// Constructs a rover object with the given configuration file and lcm
// object with which to use for communications.
Rover::Rover(const rapidjson::Document& config, lcm::LCM& lcmObject)
        : mConfig(config), mLcmObject(lcmObject),
          mBearingPid(PidLoop(config["bearingPid"]["kP"].GetDouble(),
                              config["bearingPid"]["kI"].GetDouble(),
                              config["bearingPid"]["kD"].GetDouble())
                              .withMaxInput(360.0)
                              .withThreshold(mConfig["navThresholds"]["turningBearing"].GetDouble())),
          mLongMeterInMinutes(-1) {
} // Rover(

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
} // drive()

bool Rover::drive(double distance, double bearing, double threshold, double dt) {
    if (distance < threshold) {
        return true;
    }
    TargetBearing targetBearingLCM = {bearing};
    std::string const& targetBearingChannel = mConfig["lcmChannels"]["targetBearingChannel"].GetString();
    mLcmObject.publish(targetBearingChannel, &targetBearingLCM);
    if (turn(bearing, dt)) {
        double destinationBearing = mod(bearing, 360);
        double turningEffort = mBearingPid.update(mOdometry.bearing_deg, destinationBearing, dt);
        // When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        // if we need to turn clockwise, turning effort will be positive, so leftVel will be 1, and rightVel will be in between 0 and 1
        // if we need to turn ccw, turning effort will be negative, so rightVel will be 1 and leftVel will be in between 0 and 1
        // TODO: use std::clamp
//        double leftVel = std::clamp(1.0 + turningEffort, 0.0, 1.0);
//        double rightVel = std::clamp(1.0 - turningEffort, 0.0, 1.0);
        double leftVel = std::min(1.0, std::max(0.0, 1.0 + turningEffort));
        double rightVel = std::min(1.0, std::max(0.0, 1.0 - turningEffort));
        publishAutonDriveCmd(leftVel, rightVel);
    }

    return false;
} // drive()

// Turn to a global point
bool Rover::turn(Odometry const& destination, double dt) {
    double bearing = estimateBearing(mOdometry, destination);
    return turn(bearing, dt);
} // turn()


// Turn to an absolute bearing
bool Rover::turn(double absoluteBearing, double dt) {
    if (mBearingPid.isOnTarget(mOdometry.bearing_deg, absoluteBearing)) {
        return true;
    }
    double turningEffort = mBearingPid.update(mOdometry.bearing_deg, absoluteBearing, dt);
    // To turn in place we apply +turningEffort, -turningEffort on either side and make sure they're both within [-1, 1]
    double leftVel = std::max(std::min(1.0, +turningEffort), -1.0);
    double rightVel = std::max(std::min(1.0, -turningEffort), -1.0);
    publishAutonDriveCmd(leftVel, rightVel);
    return false;
} // turn()

void Rover::stop() {
    publishAutonDriveCmd(0.0, 0.0);
} // stop()

// Calculates the conversion from minutes to meters based on the
// rover's current latitude.
double Rover::longMeterInMinutes() const {
    if (mLongMeterInMinutes <= 0.0) {
        throw std::runtime_error("Invalid conversion");
    }
    return mLongMeterInMinutes;
}

// Gets the rover's turning pid object.
PidLoop& Rover::bearingPid() {
    return mBearingPid;
} // bearingPid()

void Rover::publishAutonDriveCmd(const double leftVel, const double rightVel) {
    AutonDriveControl driveControl{
            .left_percent_velocity = leftVel,
            .right_percent_velocity = rightVel
    };
    std::string autonDriveControlChannel = mConfig["lcmChannels"]["autonDriveControlChannel"].GetString();
    mLcmObject.publish(autonDriveControlChannel, &driveControl);
}

// Gets a reference to the rover's current navigation state.
NavState const& Rover::currentState() const {
    return mCurrentState;
} // currentState()

// Gets a reference to the rover's current auton state.
AutonState const& Rover::autonState() const {
    return mAutonState;
} // autonState()

// Gets a reference to the rover's current odometry information.
Odometry const& Rover::odometry() const {
    return mOdometry;
} // odometry()

void Rover::setAutonState(AutonState state) {
    mAutonState = state;
}

void Rover::setOdometry(const Odometry& odometry) {
    mOdometry = odometry;
}

void Rover::setState(NavState state) {
    mCurrentState = state;
}

void Rover::setLongMeterInMinutes(double l) {
    mLongMeterInMinutes = l;
}