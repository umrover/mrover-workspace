#ifndef DIAMOND_GATE_SEARCH_HPP
#define DIAMOND_GATE_SEARCH_HPP

#include "gateStateMachine.hpp"

class DiamondGateSearch : public GateStateMachine
{
public:
    DiamondGateSearch( std::weak_ptr<StateMachine> stateMachine, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig );

    ~DiamondGateSearch() override;

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch() override;
};

#endif // DIAMOND_GATE_SEARCH_HPP
