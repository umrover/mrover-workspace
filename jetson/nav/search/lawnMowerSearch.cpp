#include "lawnMowerSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>

void LawnMower::initializeSearch(shared_ptr<Rover> rover, const rapidjson::Document& config, const double visionDistance) {
    const double searchBailThresh = config["search"]["bailThresh"].GetDouble();

    mSearchPoints.clear();

    mSearchPointMultipliers.clear();
    // mSearchPointMultipliers.push_back( pair<short, short> (  0, 0 ) );
    mSearchPointMultipliers.emplace_back(0, 1);
    mSearchPointMultipliers.emplace_back(-1, 1);
    mSearchPointMultipliers.emplace_back(-1, 0);
    mSearchPointMultipliers.emplace_back(-2, 0);


    while (fabs(mSearchPointMultipliers[0].first * visionDistance) < searchBailThresh) {
        for (auto& mSearchPointMultiplier: mSearchPointMultipliers) {
            Odometry nextSearchPoint = rover->odometry();

            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                                          (mSearchPointMultiplier.first * visionDistance * LAT_METER_IN_MINUTES);
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                                           (mSearchPointMultiplier.second * (2 * searchBailThresh) * rover->longMeterInMinutes());

            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = (totalLatitudeMinutes - (((int) totalLatitudeMinutes) / 60) * 60);
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = (totalLongitudeMinutes - (((int) totalLongitudeMinutes) / 60) * 60);

            mSearchPointMultiplier.first -= 2;
            mSearchPoints.push_back(nextSearchPoint);
        }
    }
    insertIntermediatePoints();
} // initializeSearch()
