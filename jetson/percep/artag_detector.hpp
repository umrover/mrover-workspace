#pragma once

#include <vector>
#include "perception.hpp"
#include <rapidjson/error/en.h>

using namespace std;
using namespace cv;

struct Tag {
    Point2f loc;
    int id;
};

class TagDetector {
   private:
    Ptr<cv::aruco::Dictionary> alvarDict;
    Ptr<cv::aruco::DetectorParameters> alvarParams;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Mat rgb;
    //reference to config file
    const rapidjson::Document& mRoverConfig;


   public:
    TagDetector(const rapidjson::Document& config);                                                                  //constructor loads dictionary data from file
    Point2f getAverageTagCoordinateFromCorners(const vector<Point2f> &corners);     //takes detected AR tag and finds center coordinate for use with ZED
    pair<Tag, Tag> findARTags(Mat &src, Mat &depth_src, Mat &rgb);                  //detects AR tags in a given Mat
    double getAngle(float xPixel, float wPixel);                                    //finds the angle from center given pixel coordinates
};
