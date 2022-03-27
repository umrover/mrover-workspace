#pragma once

#include <utility>

#include "searchStateMachine.hpp"

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class LawnMower : public SearchStateMachine {
public:
    LawnMower(weak_ptr<StateMachine> stateMachine, shared_ptr<Rover> rover, const rapidjson::Document& config)
            : SearchStateMachine(move(stateMachine), move(rover), config) {}

    // Initializes the search point multipliers to be the innermost loop
    // of the search.
    void initializeSearch(shared_ptr<Rover> rover, const rapidjson::Document& config, double pathWidth) override;
};
