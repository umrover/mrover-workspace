#pragma once

#include <utility>

#include "searchStateMachine.hpp"

class SpiralOut : public SearchStateMachine {
public:
    SpiralOut(weak_ptr<StateMachine> stateMachine, shared_ptr<Rover> rover, const rapidjson::Document& config)
            : SearchStateMachine(move(stateMachine), move(rover), config) {}

    // Initializes the search point multipliers to be the innermost loop
    // of the search.
    void initializeSearch(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig, double pathWidth) override;
};
