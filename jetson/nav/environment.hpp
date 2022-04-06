#pragma once

#include <vector>

#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/TargetList.hpp"


using namespace rover_msgs;

class Environment {
private:
    Obstacle mObstacle{0.0, 0.0, -1.0};
    TargetList mTargets{};

public:
    Environment();

    Obstacle getObstacle();

    void setObstacle(Obstacle const& obstacle);

    TargetList getTargets();

    void setTargets(TargetList const& targets);
};