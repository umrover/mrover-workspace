#pragma once

#include <utility>

#include "searchStateMachine.hpp"

class SpiralOut : public SearchStateMachine {
public:
    SpiralOut(weak_ptr<StateMachine> sm, shared_ptr<Rover> rover, const rapidjson::Document& config)
            : SearchStateMachine(move(sm), config) {}

    // Initializes the search point multipliers to be the innermost loop
    // of the search.
    void initializeSearch(const rapidjson::Document& roverConfig, double pathWidth) override;
};
