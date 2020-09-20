#ifndef SPIRAL_IN_SEARCH_HPP
#define SPIRAL_IN_SEARCH_HPP

#include "searchStateMachine.hpp"

/*************************************************************************/
/* SpiralIn Search */
/*************************************************************************/
class SpiralIn : public SearchStateMachine
{
public:
    SpiralIn( StateMachine* stateMachine_ )
    : SearchStateMachine(stateMachine_) {}

    ~SpiralIn();

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig, const double pathWidth );
};

#endif //SPIRAL_IN_SEARCH_HPP