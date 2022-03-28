#include "spiralInSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void SpiralIn::initializeSearch(const rapidjson::Document& roverConfig, double visionDistance) {
    mSearchPoints.clear();

    mSearchPointMultipliers.clear();
    mSearchPointMultipliers.emplace_back(-1, 0);
    mSearchPointMultipliers.emplace_back(-1, 1);
    mSearchPointMultipliers.emplace_back(1, 1);
    mSearchPointMultipliers.emplace_back(1, -1);

    while (mSearchPointMultipliers[0].second * visionDistance < roverConfig["search"]["bailThresh"].GetDouble()) {
        for (auto& mSearchPointMultiplier: mSearchPointMultipliers) {
            Odometry nextSearchPoint = mStateMachine.lock()->getCourseState()->getRemainingWaypoints().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                                          (mSearchPointMultiplier.first * visionDistance * LAT_METER_IN_MINUTES);
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                                           (mSearchPointMultiplier.second * visionDistance * mStateMachine.lock()->getRover()->longMeterInMinutes());
            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = (totalLatitudeMinutes - (((int) totalLatitudeMinutes) / 60) * 60);
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = (totalLongitudeMinutes - (((int) totalLongitudeMinutes) / 60) * 60);

            mSearchPoints.push_back(nextSearchPoint);

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
        }
    }
    insertIntermediatePoints();
    //TODO Reverse Deque. Not using this search though...
} // initializeSearch()