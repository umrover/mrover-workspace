#ifndef LAWN_MOWER_SEARCH_HPP
#define LAWN_MOWER_SEARCH_HPP

#include "searchStateMachine.hpp"

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class LawnMower : public SearchStateMachine
{
public:
    LawnMower( StateMachine* stateMachine_, Rover* rover, const rapidjson::Document& roverConfig )
    : SearchStateMachine( stateMachine_, rover, roverConfig ) {}

    ~LawnMower();

    // Initializes the search point multipliers to be the intermost loop
    // of the search.
    void initializeSearch( Rover* rover, const rapidjson::Document& roverConfig, const double pathWidth );
};

#endif //LAWN_MOWER_SEARCH_HPP
