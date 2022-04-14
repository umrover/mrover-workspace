#include <cmath>

#include "utilities.hpp"
#include "environment.hpp"
#include <iostream>
#include <string>

Environment::Environment(const rapidjson::Document& config) :
        mConfig(config),
<<<<<<< HEAD
        mTargetLeft(3, 20, {-1, -1, -1}),
        mTargetRight(3, 20, {-1, -1, -1})
        {}
=======
        mLeftBearingFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()),
        mRightBearingFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()),
        mLeftDistanceFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()),
        mRightDistanceFilter(config["gate"]["filterSize"].GetInt(), config["gate"]["filterProportion"].GetDouble()) {}
>>>>>>> 7eab089391ae8331cab008518d7aa1a77b2b721e

void Environment::setObstacle(Obstacle const& obstacle) {
    mObstacle = obstacle;
}

Obstacle Environment::getObstacle() {
    return mObstacle;
}

Target Environment::getLeftTarget() {
<<<<<<< HEAD
    return mTargetLeft.get();
}

Target Environment::getRightTarget() {
    return mTargetRight.get();
}

void Environment::setBaseGateID(int baseGateId) {
    mBaseGateId = baseGateId;
}

int Environment::getBaseGateID() const {
    return mBaseGateId;
}

void Environment::setTargets(TargetList const& targets) {
    Target const& leftTargetRaw = targets.targetList[0];
    Target const& rightTargetRaw = targets.targetList[1];
    //std::cout << (leftTargetRaw.distance == -1 || leftTargetRaw.id == -1) << std::endl;
    mTargetLeft.put((leftTargetRaw.distance != -1 && leftTargetRaw.id != -1), leftTargetRaw);
    mTargetRight.put((rightTargetRaw.distance != -1 && rightTargetRaw.id != -1), rightTargetRaw);
}

void Environment::updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course) {
    mHasNewPostUpdate = false;
    if (rover->autonState().is_auton) {
        bool rightReady = mTargetRight.isValid();
        bool leftReady = mTargetLeft.isValid();
        Target const& leftTarget = mTargetLeft.get();
        Target const& rightTarget = mTargetRight.get();
        double currentBearing = rover->odometry().bearing_deg;
        if (rightReady) {
            if (rightTarget.id == mBaseGateId) {
                mPostOne = createOdom(rover->odometry(), currentBearing + rightTarget.bearing, rightTarget.distance, rover);
                mHasPostOne = true;
                mHasNewPostUpdate = true;
            }
            if (rightTarget.id == mBaseGateId + 1) {
                mPostTwo = createOdom(rover->odometry(), currentBearing + rightTarget.bearing, rightTarget.distance, rover);
                mHasPostTwo = true;
                mHasNewPostUpdate = true;
            }
        }
        if (leftReady) {
            if (leftTarget.id == mBaseGateId) {
                mPostOne = createOdom(rover->odometry(), currentBearing + leftTarget.bearing, leftTarget.distance, rover);
                mHasPostOne = true;
                mHasNewPostUpdate = true;
            }
            if (leftTarget.id == mBaseGateId + 1) {
                mPostTwo = createOdom(rover->odometry(), currentBearing + leftTarget.bearing, leftTarget.distance, rover);
                mHasPostTwo = true;
                mHasNewPostUpdate = true;
=======
    return mTargetLeft;
}

Target Environment::getRightTarget() {
    return mTargetRight;
}

void Environment::setBaseGateID(int b) {
    baseGateID = b;
}

int Environment::getBaseGateID() {
    return baseGateID;
}

void Environment::setTargets(TargetList const& targets) {
    mTargetLeft = targets.targetList[0];
    mTargetRight = targets.targetList[1];
    if (mTargetLeft.id == mConfig["navThresholds"]["noTargetDist"].GetInt()) {
        mLeftBearingFilter.reset();
        mLeftDistanceFilter.reset();
    } else {
        mLeftBearingFilter.push(mTargetLeft.bearing);
        mLeftDistanceFilter.push(mTargetLeft.distance);
        mTargetLeft.bearing = mLeftBearingFilter.get();
        mTargetLeft.distance = mLeftDistanceFilter.get();
    }

    if (targets.targetList[1].id == mConfig["navThresholds"]["noTargetDist"].GetInt()) {
        mRightBearingFilter.reset();
        mRightDistanceFilter.reset();
    } else {
        std::cout << "adding right readings" << std::endl;
        mRightBearingFilter.push(mTargetRight.bearing);
        mRightDistanceFilter.push(mTargetRight.distance);
        mTargetRight.bearing = mRightBearingFilter.get();
        mTargetRight.distance = mRightDistanceFilter.get();
    }
}

void Environment::updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course) {
    if (rover->autonState().is_auton) {
        bool rightReady = isRightTargetFilterReady();
        bool leftReady = isLeftTargetFilterReady();
        double currentBearing = rover->odometry().bearing_deg;
        if (rightReady) {
//            std::cout << "right ready: " << mTargetRight.id << std::endl;
            if (mTargetRight.id == baseGateID) {
//                std::cout << "updated post 1" << std::endl;
                mPostOne = createOdom(rover->odometry(), currentBearing + mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
                mHasPostOne = true;
            }
            if (mTargetRight.id == baseGateID + 1) {
//                std::cout << "updated post 2" << std::endl;
                mPostTwo = createOdom(rover->odometry(), currentBearing + mRightBearingFilter.get(), mRightDistanceFilter.get(), rover);
                mHasPostTwo = true;
            }
        }
        if (leftReady) {
//            std::cout << "left ready: " << mTargetLeft.id << std::endl;
//            std::cout << "course waypoint id: " << course->getCurrentWaypoint().id << std::endl;
            if (mTargetLeft.id == baseGateID) {
//                std::cout << "updated post 1" << std::endl;
                mPostOne = createOdom(rover->odometry(), currentBearing + mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
                mHasPostOne = true;
            }
            if (mTargetLeft.id == baseGateID + 1) {
//                std::cout << "updated post 2" << std::endl;
                mPostTwo = createOdom(rover->odometry(), currentBearing + mLeftBearingFilter.get(), mLeftDistanceFilter.get(), rover);
                mHasPostTwo = true;
>>>>>>> 7eab089391ae8331cab008518d7aa1a77b2b721e
            }
        }
    } else {
        mHasPostOne = mHasPostTwo = false;
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes(60 / (EARTH_CIRCUM * cosine / 360));
    }
}

<<<<<<< HEAD
bool Environment::hasNewPostUpdate() const {
    return mHasNewPostUpdate;
}

=======
>>>>>>> 7eab089391ae8331cab008518d7aa1a77b2b721e
bool Environment::hasGateLocation() const {
    return mHasPostOne && mHasPostTwo;
}

bool Environment::hasPostOneLocation() const {
    return mHasPostOne;
}

bool Environment::hasPostTwoLocation() const {
    return mHasPostTwo;
}

Odometry Environment::getPostOneLocation() {
    return mPostOne;
}

Odometry Environment::getPostTwoLocation() {
    return mPostTwo;
}

// Offset of the post in our linearized cartesian space.
Vector2d Environment::getPostOneOffsetInCartesian(Odometry cur) {
    double bearing = degreeToRadian(estimateBearing(cur, mPostOne));
    double distance = estimateDistance(cur, mPostOne);
    return {distance * cos(bearing), distance * sin(bearing)};
}

Vector2d Environment::getPostTwoOffsetInCartesian(Odometry cur) {
    double bearing = degreeToRadian(estimateBearing(cur, mPostTwo));
    double distance = estimateDistance(cur, mPostTwo);
    return {distance * cos(bearing), distance * sin(bearing)};
}

<<<<<<< HEAD
std::optional<Target> Environment::tryGetTargetWithId(int32_t id) {
    if (mTargetLeft.get().id == id && mTargetLeft.get().distance > 0.0) {
        return {mTargetLeft.get()};
    } else if (mTargetRight.get().id == id && mTargetRight.get().distance > 0.0) {
        return {mTargetRight.get()};
=======
bool Environment::areTargetFiltersReady() const {
    return mLeftDistanceFilter.ready() && mRightDistanceFilter.ready() && mLeftBearingFilter.ready() && mRightBearingFilter.ready();
}

bool Environment::isLeftTargetFilterReady() const {
    return mLeftDistanceFilter.ready() && mLeftBearingFilter.ready();
}

bool Environment::isRightTargetFilterReady() const {
    return mRightDistanceFilter.ready() && mRightBearingFilter.ready();
}

std::optional<Target> Environment::tryGetTargetWithId(int32_t id) {
    if (mTargetLeft.id == id && mTargetLeft.distance > 0.0) {
        return {mTargetLeft};
    } else if (mTargetRight.id == id && mTargetRight.distance > 0.0) {
        return {mTargetRight};
>>>>>>> 7eab089391ae8331cab008518d7aa1a77b2b721e
    }
    return std::nullopt;
}
