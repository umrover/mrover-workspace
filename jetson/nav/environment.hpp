#pragma once

#include <vector>
#include <optional>

#include <eigen3/Eigen/Core>

#include "rover.hpp"
#include "filter.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/TargetList.hpp"

using Eigen::Vector2d;
using namespace rover_msgs;

class Rover;

class Environment {
private:
    Obstacle mObstacle{0.0, 0.0, -1.0};

    // The rover's current target information from computer
    // vision.
    Target mTargetLeft{-1.0, 0.0, -1};
    Target mTargetRight{-1.0, 0.0, -1};

    // Reference to config variables
    const rapidjson::Document& mConfig;

    Filter<double> mLeftBearingFilter, mRightBearingFilter, mLeftDistanceFilter, mRightDistanceFilter;

    Odometry mPostOne{}, mPostTwo{};

    bool hasPostOne = false, hasPostTwo = false;

    int baseGateID;

public:
    explicit Environment(const rapidjson::Document& config);

    Obstacle getObstacle();

    void setObstacle(Obstacle const& obstacle);

    void setBaseGateID(int b);

    int getBaseGateID();

    Target getLeftTarget();

    Target getRightTarget();

    Odometry getPostOneLocation();

    Odometry getPostTwoLocation();

    Vector2d getPostOneRelative(Odometry cur);

    Vector2d getPostTwoRelative(Odometry cur);

    void setTargets(TargetList const& targets);

    void updateTargets(std::shared_ptr<Rover> const& rover, std::shared_ptr<CourseProgress> const& course);

    [[nodiscard]] bool hasGateLocation() const;

    [[nodiscard]] bool hasPostOneLocation() const;

    [[nodiscard]] bool hasPostTwoLocation() const;

    [[nodiscard]] bool areTargetFiltersReady() const;

    [[nodiscard]] bool isLeftTargetFilterReady() const;

    [[nodiscard]] bool isRightTargetFilterReady() const;

    std::optional<Target> tryGetTargetWithId(int32_t id);
};