#pragma once

#include "searchStateMachine.hpp"

/*************************************************************************/
/* SpiralIn Search */
/*************************************************************************/
class SpiralIn : public SearchStateMachine {
public:
    SpiralIn(weak_ptr<StateMachine> stateMachine_, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
            : SearchStateMachine(stateMachine_, rover, roverConfig) {}

    ~SpiralIn();

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig, const double pathWidth);
};
