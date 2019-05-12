#ifndef LAWN_MOWER_SEARCH_HPP
#define LAWN_MOWER_SEARCH_HPP

#include "searchStateMachine.hpp"

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class LawnMower : public SearchStateMachine
{
public:
    LawnMower( StateMachine* stateMachine_ )
    : SearchStateMachine(stateMachine_) {}

    ~LawnMower();

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch( Rover* phoebe, const rapidjson::Document& roverConfig, const double pathWidth );
};

#endif //LAWN_MOWER_SEARCH_HPP
