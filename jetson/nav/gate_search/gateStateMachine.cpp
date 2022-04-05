#include "gateStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "./gate_search/circleGateSearch.hpp"
#include <cmath>
#include <iostream>
#include <utility>

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig)
        : mStateMachine(move(stateMachine)), mRoverConfig(roverConfig) {}

GateStateMachine::~GateStateMachine() = default;

// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    auto rover = mStateMachine.lock()->getRover();
    switch (rover->currentState()) {
        case NavState::Gate: {
            break;
        }
        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<CircleGateSearch>(sm, roverConfig);
} // GateFactory()