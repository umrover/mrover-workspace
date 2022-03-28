#pragma once

#include <utility>

#include "searchStateMachine.hpp"

/*************************************************************************/
/* SpiralIn Search */
/*************************************************************************/
class SpiralIn : public SearchStateMachine {
public:
    SpiralIn(weak_ptr<StateMachine> sm, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
            : SearchStateMachine(move(sm), move(rover), roverConfig) {}

    // Initializes the search point multipliers to be the innermost loop
    // of the search.
    void initializeSearch(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig, double pathWidth) override;
};
