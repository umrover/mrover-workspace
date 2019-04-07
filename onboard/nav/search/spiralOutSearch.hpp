#ifndef SPIRAL_OUT_SEARCH_HPP
#define SPIRAL_OUT_SEARCH_HPP

#include "searchStateMachine.hpp"

class SpiralOut : public SearchStateMachine
{
public:
    SpiralOut( StateMachine* stateMachine_ )
    : SearchStateMachine(stateMachine_) {}

    ~SpiralOut();

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig, const double pathWidth );
};

#endif //SPIRAL_OUT_SEARCH_HPP