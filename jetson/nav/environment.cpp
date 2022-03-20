//
// Created by quintin on 3/20/22.
//

#include "environment.hpp"

Environment::Environment() = default;

void Environment::setObstacle(Obstacle const& obstacle) {
    mObstacle = obstacle;
}

Obstacle Environment::getObstacle() {
    return mObstacle;
}

TargetList Environment::getTargets() {
    return mTargets;
}

void Environment::setTargets(TargetList const& targets) {
    mTargets = targets;
}
