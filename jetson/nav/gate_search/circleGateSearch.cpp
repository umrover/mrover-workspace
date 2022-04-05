#include "circleGateSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <utility>

CircleGateSearch::CircleGateSearch(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig)
        : GateStateMachine(move(sm), roverConfig) {}

CircleGateSearch::~CircleGateSearch() = default;

void CircleGateSearch::initializeSearch() {

} // initializeSearch()
