#include <cmath>

#include "utilities.hpp"
#include "environment.hpp"
#include <iostream>
#include <string>

Environment::Environment(const rapidjson::Document& config) :
        mConfig(config),
        mTargetLeft(3, 20, {-1, -1, -1}),
        mTargetRight(3, 20, {-1, -1, -1})
        {}

void Environment::setObstacle(Obstacle const& obstacle) {
    mObstacle = obstacle;
}

Obstacle Environment::getObstacle() {
    return mObstacle;
}

Target Environment::getLeftTarget() {
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
            }
        }
    } else {
        mHasPostOne = mHasPostTwo = false;
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes(60 / (EARTH_CIRCUM * cosine / 360));
    }
}

bool Environment::hasNewPostUpdate() const {
    return mHasNewPostUpdate;
}

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

std::optional<Target> Environment::tryGetTargetWithId(int32_t id) {
    if (mTargetLeft.get().id == id && mTargetLeft.get().distance > 0.0) {
        return {mTargetLeft.get()};
    } else if (mTargetRight.get().id == id && mTargetRight.get().distance > 0.0) {
        return {mTargetRight.get()};
    }
    return std::nullopt;
}
