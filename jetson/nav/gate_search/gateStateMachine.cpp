#include "gateStateMachine.hpp"

#include <utility>
#include <iostream>

#include "utilities.hpp"
#include "environment.hpp"
#include "stateMachine.hpp"
#include "./gate_search/circleGateSearch.hpp"

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig) :
        mStateMachine(move(stateMachine)),
        mRoverConfig(roverConfig),
        mTargetFilter(roverConfig["gate"]["filterSize"].GetInt()) {
}

GateStateMachine::~GateStateMachine() = default;


// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    auto rover = mStateMachine.lock()->getRover();
    switch (rover->currentState()) {
        case NavState::GateMakePath: {
            std::shared_ptr<Environment> env = mStateMachine.lock()->getEnv();
            TargetList targets = env->getTargets();
            rover->drive(0.0, 0.0, true);
            return NavState::GateMakePath;
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