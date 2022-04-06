#pragma once

#include <utility>

#include "searchStateMachine.hpp"

class SearchFromPathFile : public SearchStateMachine {
private:
    std::string mPath;

public:
    SearchFromPathFile(std::weak_ptr<StateMachine> sm, const rapidjson::Document& config, std::string path)
            : SearchStateMachine(move(sm), config), mPath(move(path)) {}

    void initializeSearch(const rapidjson::Document& roverConfig, double pathWidth) override;
};

