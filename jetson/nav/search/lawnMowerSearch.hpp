#ifndef LAWN_MOWER_SEARCH_HPP
#define LAWN_MOWER_SEARCH_HPP

#include "searchStateMachine.hpp"

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class LawnMower : public SearchStateMachine {
public:
    LawnMower(std::weak_ptr<StateMachine> stateMachine_, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
            : SearchStateMachine(stateMachine_, rover, roverConfig) {}

    ~LawnMower() override;

    // Initializes the search point multipliers to be the intermost loop
    // of the search.
    void initializeSearch(std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig, double pathWidth);
};

#endif //LAWN_MOWER_SEARCH_HPP
