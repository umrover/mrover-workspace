#pragma once

#include "gateStateMachine.hpp"

class CircleGateSearch : public GateStateMachine {
public:
    CircleGateSearch(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig);

    ~CircleGateSearch() override;

    void initializeSearch() override;
};
