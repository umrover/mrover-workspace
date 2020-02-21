#ifndef DIAMOND_GATE_SEARCH_HPP
#define DIAMOND_GATE_SEARCH_HPP

#include "gateStateMachine.hpp"

class DiamondGateSearch : public GateStateMachine
{
public:
    DiamondGateSearch( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig );

    virtual ~DiamondGateSearch() override;

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    virtual void initializeSearch() override;
};

#endif // DIAMOND_GATE_SEARCH_HPP
