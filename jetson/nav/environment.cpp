#include <cmath>

#include "utilities.hpp"
#include "environment.hpp"
#include <iostream>
#include <string>

Environment::Environment(const rapidjson::Document& config) :
        mConfig(config),
        mTargetLeft(3, 1, {-1, -1, -1}),
        mTargetRight(3, 1, {-1, -1, -1}),
        mPostOneLat(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mPostOneLong(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mPostTwoLat(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mPostTwoLong(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()) {}

void Environment::setObstacle(Obstacle const& obstacle) {
    mObstacle = obstacle;
}

Obstacle Environment::getObstacle() {
    return mObstacle;
}

Target Environment::getLeftTarget() const {
    return mTargetLeft.get();
}

Target Environment::getRightTarget() const {
    return mTargetRight.get();
}

void Environment::setBaseGateID(int baseGateId) {
    mBaseGateId = baseGateId;
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
        bool isRightValid = mTargetRight.isValid();
        bool isLeftValid = mTargetLeft.isValid();
        Target const& leftTarget = mTargetLeft.get();
        Target const& rightTarget = mTargetRight.get();
        double currentBearing = rover->odometry().bearing_deg;
        if (isRightValid) {
            Odometry postOdom = createOdom(rover->odometry(), currentBearing + rightTarget.bearing, rightTarget.distance, rover);
            if (rightTarget.id == mBaseGateId) {
//                std::cout << "updating post 1 r" << std::endl;
                mPostOneLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostOneLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
            if (rightTarget.id == mBaseGateId + 1) {
//                std::cout << "updating post 2 r" << std::endl;
                mPostTwoLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostTwoLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
        }
        if (isLeftValid) {
            Odometry postOdom = createOdom(rover->odometry(), currentBearing + leftTarget.bearing, leftTarget.distance, rover);
            if (leftTarget.id == mBaseGateId) {
//                std::cout << "updating post 1 l" << std::endl;
                mPostOneLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostOneLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
            if (leftTarget.id == mBaseGateId + 1) {
//                std::cout << "updating post 2 l" << std::endl;
                mPostTwoLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostTwoLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
        }
    } else {
        mPostOneLat.reset();
        mPostOneLong.reset();
        mPostTwoLat.reset();
        mPostTwoLong.reset();
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes(60 / (EARTH_CIRCUM * cosine / 360));
    }
}

bool Environment::hasNewPostUpdate() const {
    return mHasNewPostUpdate;
}

bool Environment::hasGateLocation() const {
    return hasPostOneLocation() && hasPostTwoLocation();
}

bool Environment::hasPostOneLocation() const {
    return mPostOneLat.ready();
}

bool Environment::hasPostTwoLocation() const {
    return mPostTwoLat.ready();
}

Odometry createOdom(double latitude, double longitude) {
    double latitudeDeg;
    double longitudeDeg;
    double latitudeMin = std::modf(latitude, &latitudeDeg);
    double longitudeMin = std::modf(longitude, &longitudeDeg);
    latitudeMin *= 60.0;
    longitudeMin *= 60.0;
    return Odometry{
            static_cast<int32_t>(latitudeDeg), latitudeMin,
            static_cast<int32_t>(longitudeDeg), longitudeMin
    };
}

Odometry Environment::getPostOneLocation() const {
    return createOdom(mPostOneLat.get(), mPostOneLong.get());
}

Odometry Environment::getPostTwoLocation() const {
    return createOdom(mPostTwoLat.get(), mPostTwoLong.get());
}

Vector2d getOffsetInCartesian(Odometry current, Odometry target) {
    double bearing = degreeToRadian(estimateBearing(current, target));
    double distance = estimateDistance(current, target);
    return {distance * cos(bearing), distance * sin(bearing)};
}

// Offset of the post in our linearized cartesian space.
Vector2d Environment::getPostOneOffsetInCartesian(Odometry cur) const {
    return getOffsetInCartesian(cur, getPostOneLocation());
}

Vector2d Environment::getPostTwoOffsetInCartesian(Odometry cur) const {
    return getOffsetInCartesian(cur, getPostTwoLocation());

}

std::optional<Target> Environment::tryGetTargetWithId(int32_t id) const {
    if (mTargetLeft.get().id == id && mTargetLeft.get().distance > 0.0) {
        return {mTargetLeft.get()};
    } else if (mTargetRight.get().id == id && mTargetRight.get().distance > 0.0) {
        return {mTargetRight.get()};
    }
    return std::nullopt;
}
