#ifndef GIMBAL_SPIRAL_OUT_SEARCH_HPP
#define GIMBAL_SPIRAL_OUT_SEARCH_HPP

#include "searchStateMachine.hpp"

/*************************************************************************/
/* GimbalSpiralOut Search */
/*************************************************************************/
class GimbalSpiralOut : public SearchStateMachine
{
public:
    GimbalSpiralOut( StateMachine* stateMachine_ )
    : SearchStateMachine(stateMachine_) {}

    ~GimbalSpiralOut();

    // Initializes the search point multipliers to be the intermost loop
    // of the search.
    void initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig, const double pathWidth );
};

#endif //GIMBAL_SPIRAL_OUT_SEARCH_HPP
