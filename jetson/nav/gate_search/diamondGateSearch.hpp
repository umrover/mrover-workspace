#pragma once

#include "gateStateMachine.hpp"

class DiamondGateSearch : public GateStateMachine {
public:
    DiamondGateSearch(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig);

    ~DiamondGateSearch() override;

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch() override;
};
