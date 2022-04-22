#pragma once

#include <vector>

#include "perception.hpp"
#include "rover_msgs/Target.hpp"

struct Tag {
    cv::Point2f loc;
    int id;
};

class TagDetector {
private:
    cv::Ptr<cv::aruco::Dictionary> mAlvarDict;
    cv::Ptr<cv::aruco::DetectorParameters> mAlvarParams;
    std::vector<int> mIds;
    std::vector<std::vector<cv::Point2f>> mCorners;
    cv::Mat mRgb;

public:
    //Constants:
    int BUFFER_ITERATIONS;
    int MARKER_BORDER_BITS;
    bool DO_CORNER_REFINEMENT;
    double POLYGONAL_APPROX_ACCURACY_RATE;
    int MM_PER_M;
    int DEFAULT_TAG_VAL;

    //constructor loads alvar dictionary data from file that defines tag bit configurations
    explicit TagDetector(const rapidjson::Document& mRoverConfig);

    //takes detected AR tag and finds center coordinate for use with ZED                                                                 
    static cv::Point2f getAverageTagCoordinateFromCorners(const std::vector<cv::Point2f>& corners);

    //detects AR tags in a given Mat
    std::pair<Tag, Tag> findARTags(cv::Mat& src, cv::Mat& depth_src, cv::Mat& rgb);

    //if AR tag found, updates distance, bearing, and id
    void updateDetectedTagInfo(rover_msgs::Target* outTags, std::pair<Tag, Tag> const& tagPair, cv::Mat const& depth_img, cv::Mat const& xyz_img) const;
};
