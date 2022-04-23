#include <cmath>

#include "utilities.hpp"
#include "environment.hpp"

Environment::Environment(const rapidjson::Document& config) :
        mConfig(config),
        mTargetLeft(mConfig["navThresholds"]["cacheHitMax"].GetInt(), mConfig["navThresholds"]["cacheMissMax"].GetInt(), {-1, -1, -1}),
        mTargetRight(mConfig["navThresholds"]["cacheHitMax"].GetInt(), mConfig["navThresholds"]["cacheMissMax"].GetInt(), {-1, -1, -1}),
        mPostOneLat(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mPostOneLong(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mPostTwoLat(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mPostTwoLong(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mLeftTargetBearing(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mLeftTargetDistance(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mRightTargetBearing(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()),
        mRightTargetDistance(mConfig["gate"]["filterSize"].GetInt(), mConfig["gate"]["filterProportion"].GetDouble()) {}

/***
 * @param targets New target data from perception to update our filters.
 */
void Environment::setTargets(TargetList const& targets) {
    Target const& leftTargetRaw = targets.targetList[0];
    Target const& rightTargetRaw = targets.targetList[1];
    mTargetLeft.put((leftTargetRaw.distance != -1 && leftTargetRaw.id != -1), leftTargetRaw);
    mTargetRight.put((rightTargetRaw.distance != -1 && rightTargetRaw.id != -1), rightTargetRaw);

    if (mTargetLeft.isValid()){
        mLeftTargetBearing.push(mTargetLeft.get().bearing);
        mLeftTargetDistance.push(mTargetLeft.get().distance);
    }
    if (mTargetRight.isValid()){
        mRightTargetBearing.push(mTargetRight.get().bearing);
        mRightTargetDistance.push(mTargetRight.get().distance);
    }
    
}

/***
 * Update our estimate for where the post is with our current filtered values.
 * This takes care of matching left/right targets in camera screen space to their actual IDs.
 * We have to convert from degrees and minutes to just degrees since that is how @c Odometry was designed.
 */
void Environment::updatePost(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course) {
    mHasNewPostUpdate = false;
    if (rover->autonState().is_auton) {
        bool isRightValid = mTargetRight.isValid();
        bool isLeftValid = mTargetLeft.isValid();
        Target const& leftTarget = mTargetLeft.get();
        Target const& rightTarget = mTargetRight.get();
        double currentBearing = rover->odometry().bearing_deg;
        // TODO: common logic, refactor out
        if (isRightValid) {
            Odometry postOdom = createOdom(rover->odometry(), currentBearing + rightTarget.bearing, rightTarget.distance, rover);
            if (rightTarget.id == mBaseGateId) {
                mPostOneLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostOneLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
            if (rightTarget.id == mBaseGateId + 1) {
                mPostTwoLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostTwoLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
        }
        if (isLeftValid) {
            Odometry postOdom = createOdom(rover->odometry(), currentBearing + leftTarget.bearing, leftTarget.distance, rover);
            if (leftTarget.id == mBaseGateId) {
                mPostOneLat.push(postOdom.latitude_deg + postOdom.latitude_min / 60.0);
                mPostOneLong.push(postOdom.longitude_deg + postOdom.longitude_min / 60.0);
                mHasNewPostUpdate = true;
            }
            if (leftTarget.id == mBaseGateId + 1) {
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
        mTargetLeft.reset();
        mTargetRight.reset();
        // TODO: move outside of this function
        double cosine = cos(degreeToRadian(rover->odometry().latitude_deg, rover->odometry().latitude_min));
        rover->setLongMeterInMinutes(60 / (EARTH_CIRCUM * cosine / 360));
    }
}

void Environment::setObstacle(Obstacle const& obstacle) {
    mObstacle = obstacle;
}

Obstacle Environment::getObstacle() {
    return mObstacle;
}

Target Environment::getLeftTarget() const {
    return {mLeftTargetDistance.get(), mLeftTargetBearing.get(), mTargetLeft.get().id};//mTargetLeft.get();
}

Target Environment::getRightTarget() const {
    return {mRightTargetDistance.get(), mRightTargetBearing.get(), mTargetRight.get().id};//mTargetRight.get();
}

void Environment::setBaseGateID(int baseGateId) {
    mBaseGateId = baseGateId;
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

Odometry Environment::getPostOneLocation() const {
    return createOdom(mPostOneLat.get(), mPostOneLong.get());
}

Odometry Environment::getPostTwoLocation() const {
    return createOdom(mPostTwoLat.get(), mPostTwoLong.get());
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
