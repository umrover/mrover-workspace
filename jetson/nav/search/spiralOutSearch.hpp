#ifndef SPIRAL_OUT_SEARCH_HPP
#define SPIRAL_OUT_SEARCH_HPP

#include "searchStateMachine.hpp"

class SpiralOut : public SearchStateMachine {
public:
    SpiralOut(std::weak_ptr<StateMachine> stateMachine_, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
            : SearchStateMachine(stateMachine_, rover, roverConfig) {}

    ~SpiralOut() override;

    // Initializes the search ponit multipliers to be the intermost loop
    // of the search.
    void initializeSearch(std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig, double pathWidth) override;
};

#endif //SPIRAL_OUT_SEARCH_HPP