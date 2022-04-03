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
    return drive(distance, bearing, false);
} // drive()

// Sends a joystick command to drive forward from the current odometry
// in the direction of bearing. The distance is used to determine how
// quickly to drive forward. This joystick command will also turn the
// rover small amounts as "course corrections". target indicates
// if the rover is driving to a target rather than a waypoint and
// determines which distance threshold to use.
// The return value indicates if the rover has arrived or if it is
// on-course or off-course.
DriveStatus Rover::drive(const double distance, const double bearing, const bool target) {
    if ((!target && distance < mRoverConfig["navThresholds"]["waypointDistance"].GetDouble()) ||
        (target && distance < mRoverConfig["navThresholds"]["targetDistance"].GetDouble())) {
        return DriveStatus::Arrived;
    }

    double destinationBearing = mod(bearing, 360);
    throughZero(destinationBearing, mOdometry.bearing_deg); // will go off course if inside if because through zero not calculated

    if (fabs(destinationBearing - mOdometry.bearing_deg) < mRoverConfig["navThresholds"]["drivingBearing"].GetDouble()) {
        double turningEffort = mBearingPid.update(mOdometry.bearing_deg, destinationBearing);
        //When we drive to a target, we want to go as fast as possible so one of the sides is fixed at one and the other is 1 - abs(turningEffort)
        //if we need to turn clockwise, turning effort will be postive, so left_vel will be 1, and right_vel will be in between 0 and 1
        //if we need to turng ccw, turning effort will be negative, so right_vel will be 1 and left_vel will be in between 0 and 1
        double left_vel = min(1.0, max(0.0, 1.0 + turningEffort));
        double right_vel = min(1.0, max(0.0, 1.0 - turningEffort));
        publishAutonDriveCmd(left_vel, right_vel);
        return DriveStatus::OnCourse;
    }
    cerr << "off course\n";
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
    double minTurningEffort = mRoverConfig["navThresholds"]["minTurningEffort"].GetDouble() * (turningEffort < 0 ? -1 : 1);
    if (isTurningAroundObstacle(mCurrentState) && fabs(turningEffort) < minTurningEffort) {
        turningEffort = minTurningEffort;
    }
    //to turn in place we apply +turningEffort, -turningEffort on either side and make sure they're both within [-1, 1]
    double left_vel = max(min(1.0, +turningEffort), -1.0);
    double right_vel = max(min(1.0, -turningEffort), -1.0);
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
        throw runtime_error("Invalid conversion");
    }
    return mLongMeterInMinutes;
}

// Gets the rover's turning pid object.
PidLoop& Rover::bearingPid() {
    return mBearingPid;
} // bearingPid()

void Rover::publishAutonDriveCmd(const double leftVel, const double rightVel) {
    AutonDriveControl driveControl{leftVel, rightVel};
    //std::cout << leftVel << " " << rightVel << std::endl;
    string autonDriveControlChannel = mRoverConfig["lcmChannels"]["autonDriveControlChannel"].GetString();
    mLcmObject.publish(autonDriveControlChannel, &driveControl);
}

bool Rover::isTurningAroundObstacle(NavState currentState) {
    return currentState == NavState::TurnAroundObs || currentState == NavState::SearchTurnAroundObs;
} // isTurningAroundObstacle()

void Rover::updateTargets(shared_ptr<Environment> const& env, shared_ptr<CourseProgress> const& course) {
    // TODO: I'm a little skeptical about how this function fits into the architecture.
    // TODO: It seems like it should be a part of the environment, not the rover.
    if (mAutonState.is_auton) {
        mTargetLeft = env->getTargets().targetList[0];
        mTargetRight = env->getTargets().targetList[1];
        // Cache Left Target if we had detected one
        if (mTargetLeft.distance != mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
            // Associate with single post
            if (mTargetLeft.id == course->getRemainingWaypoints().front().id) {
                mCountLeftHits++;
            } else {
                mCountLeftHits = 0;
            }
            // Update leftTarget if we have 3 or more consecutive hits
            if (mCountLeftHits >= 3) {
                mCTargetLeft = mTargetLeft;
                mCountLeftMisses = 0;
            }
            // Cache Right Target if we had detected one (only can see right if we see the left one, otherwise
            // results in some undefined behavior)
            if (mTargetRight.distance != mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
                mCTargetRight = mTargetRight;
                mCountRightMisses = 0;
            } else {
                mCountRightMisses++;
            }
        } else {
            mCountLeftMisses++;
            mCountRightMisses++; // need to increment since we don't see both
            mCountLeftHits = 0;
            mCountRightHits = 0;
        }

        // Check if we need to reset left cache
        if (mCountLeftMisses > mRoverConfig["navThresholds"]["cacheMissMax"].GetDouble()) {
            mCountLeftMisses = 0;
            mCountLeftHits = 0;
            // Set to empty target
            mCTargetLeft = {-1, 0, 0};
        }
        // Check if we need to reset right cache
        if (mCountRightMisses > mRoverConfig["navThresholds"]["cacheMissMax"].GetDouble()) {
            mCountRightMisses = 0;
            mCountRightHits = 0;
            // Set to empty target
            mCTargetRight = {-1, 0, 0};
        }
    } else {
        mLongMeterInMinutes = 60 / (EARTH_CIRCUM * cos(degreeToRadian(mOdometry.latitude_deg, mOdometry.latitude_min)) / 360);
    }
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

// Gets a reference to the rover's first target's current information.
Target const& Rover::leftTarget() const {
    return mTargetLeft;
} // leftTarget()

Target const& Rover::rightTarget() const {
    return mTargetRight;
} // rightTarget()

Target const& Rover::leftCacheTarget() const {
    return mCTargetLeft;
} // leftCacheTarget()

Target const& Rover::rightCacheTarget() const {
    return mCTargetRight;
} // rightCacheTarget()

int Rover::getLeftMisses() const {
    return mCountLeftMisses;
}

int Rover::getRightMisses() const {
    return mCountRightMisses;
}

int Rover::getLeftHits() const {
    return mCountLeftHits;
}

int Rover::getRightHits() const {
    return mCountRightHits;
}

void Rover::setAutonState(AutonState state) {
    mAutonState = state;
}

void Rover::setOdometry(const Odometry& odometry) {
    mOdometry = odometry;
}

void Rover::setState(NavState state) {
    mCurrentState = state;
}

void Rover::resetMisses() {
    mCountLeftMisses = 0;
    mCountRightMisses = 0;
}
