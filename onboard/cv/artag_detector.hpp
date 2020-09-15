#pragma once

#include <vector>
#include "perception.hpp"

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

   public:
    TagDetector();                                                        //constructor loads dictionary data from file
    Point2f getAverageTagCoordinateFromCorners(const vector<Point2f> &corners);  //takes detected AR tag and finds center coordinate for use with ZED
    pair<Tag, Tag> findARTags(Mat &src, Mat &depth_src, Mat &rgb);                  //detects AR tags in a given Mat
};
