#include "rover.hpp"
#include "utilities.hpp"
#include "rover_msgs/Joystick.hpp"

#include <cmath>
#include <iostream>

// Constructs a rover object with the given configuration file and lcm
// object with which to use for communications.
Rover::Rover(const rapidjson::Document& config, lcm::LCM& lcmObject)
        : mRoverConfig(config), mLcmObject(lcmObject),
          mBearingPid(config["bearingPid"]["kP"].GetDouble(),
                      config["bearingPid"]["kI"].GetDouble(),
                      config["bearingPid"]["kD"].GetDouble()),
          mLongMeterInMinutes(-1) {
} // Rover(

// Sends a joystick command to drive forward from the current odometry
// to the destination odometry. This joystick command will also turn
// the rover small amounts as "course corrections".
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive(const Odometry& destination) {
    double distance = estimateNoneuclid(mOdometry, destination);
    double bearing = calcBearing(mOdometry, destination);
    return drive(distance, bearing, mRoverConfig["navThresholds"]["waypointDistance"].GetDouble());
} // drive()

// Sends a joystick command to drive forward from the current odometry
// in the direction of bearing. The distance is used to determine how
// quickly to drive forward. This joystick command will also turn the
// rover small amounts as "course corrections". target indicates
// if the rover is driving to a target rather than a waypoint and
// determines which distance threshold to use.
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive(const double distance, const double bearing, const double threshold) {
    if (distance < threshold) {
        return DriveStatus::Arrived;
    }

    double destinationBearing = mod(bearing, 360);
    throughZero(destinationBearing, mOdometry.bearing_deg); // will go off course if inside if because through zero not calculated

    if (fabs(destinationBearing - mOdometry.bearing_deg) <
        mRoverConfig["navThresholds"]["drivingBearing"].GetDouble()) {
        double turningEffort = mBearingPid.update(mOdometry.bearing_deg, destinationBearing);
        // When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        // if we need to turn clockwise, turning effort will be positive, so left_vel will be 1, and right_vel will be in between 0 and 1
        // if we need to turn ccw, turning effort will be negative, so right_vel will be 1 and left_vel will be in between 0 and 1
        // TODO: use std::clamp
        double left_vel = std::min(1.0, std::max(0.0, 1.0 + turningEffort));
        double right_vel = std::min(1.0, std::max(0.0, 1.0 - turningEffort));
        publishAutonDriveCmd(left_vel, right_vel);
        return DriveStatus::OnCourse;
    }
//    std::cerr << "off course\n";
    return DriveStatus::OffCourse;
} // drive()

// Sends a joystick command to turn the rover toward the destination
// odometry. Returns true if the rover has finished turning, false
// otherwise.
bool Rover::turn(Odometry const& destination) {
    double bearing = calcBearing(mOdometry, destination);
    return turn(bearing);
} // turn()

// Sends a joystick command to turn the rover. The bearing is the
// absolute bearing. Returns true if the rover has finished turning, false
// otherwise.
bool Rover::turn(double bearing) {
    bearing = mod(bearing, 360);
    throughZero(bearing, mOdometry.bearing_deg);
    double turningBearingThreshold;
    if (isTurningAroundObstacle(mCurrentState)) {
        turningBearingThreshold = 0;
    } else {
        turningBearingThreshold = mRoverConfig["navThresholds"]["turningBearing"].GetDouble();
    }
    if (fabs(bearing - mOdometry.bearing_deg) <= turningBearingThreshold) {
        return true;
    }
    double turningEffort = mBearingPid.update(mOdometry.bearing_deg, bearing);
//    std::cout << "cur bearing: " << mOdometry.bearing_deg << " target bearing: " << bearing << " effort: " << turningEffort << std::endl;
    double minTurningEffort =
            mRoverConfig["navThresholds"]["minTurningEffort"].GetDouble() * (turningEffort < 0 ? -1 : 1);
    if (isTurningAroundObstacle(mCurrentState) && fabs(turningEffort) < minTurningEffort) {
        turningEffort = minTurningEffort;
    }
    //to turn in place we apply +turningEffort, -turningEffort on either side and make sure they're both within [-1, 1]
    double left_vel = std::max(std::min(1.0, +turningEffort), -1.0);
    double right_vel = std::max(std::min(1.0, -turningEffort), -1.0);
//    std::cout << left_vel << ", " << right_vel << std::endl;
    publishAutonDriveCmd(left_vel, right_vel);
    return false;
} // turn()

void Rover::stop() {
//    std::cout << "stopping" << std::endl;
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
    //std::cout << leftVel << " " << rightVel << std::endl;
    std::string autonDriveControlChannel = mRoverConfig["lcmChannels"]["autonDriveControlChannel"].GetString();
    mLcmObject.publish(autonDriveControlChannel, &driveControl);
}

bool Rover::isTurningAroundObstacle(NavState currentState) {
    return currentState == NavState::TurnAroundObs || currentState == NavState::SearchTurnAroundObs;
} // isTurningAroundObstacle()

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

void Rover::setLongMeterInMinutes(double l){
    mLongMeterInMinutes = l;
}