#ifndef DIAMOND_GATE_SEARCH_HPP
#define DIAMOND_GATE_SEARCH_HPP

#include "gateStateMachine.hpp"

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class DiamondGateSearch : public GateStateMachine
{
public:
    DiamondGateSearch( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig );

    ~DiamondGateSearch();

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch();
};

#endif // DIAMOND_GATE_SEARCH_HPP
