#pragma once

#include <vector>
#include "perception.hpp"
#include "rover_msgs/Target.hpp"

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

    /** TODO ADD these to config file
     * buffer line 155 .cpp
     * marker border bits line 38
     * docornerrefinement line 39
     * polygonal accuracy rate line 40
     * MM_PER_M line 5
     * default tag val -> replace all = -1 with = this val (note: if a number is being subtracted by one don't replace it)
     *      create and use default tag val in main.cpp lines 120,121 also
     */
    
   public:
    //constructor loads alvar dictionary data from file that defines tag bit configurations
    TagDetector(const rapidjson::Document &mRoverConfig);    
    //takes detected AR tag and finds center coordinate for use with ZED                                                                 
    Point2f getAverageTagCoordinateFromCorners(const vector<Point2f> &corners);
    //detects AR tags in a given Mat     
    pair<Tag, Tag> findARTags(Mat &src, Mat &depth_src, Mat &rgb);    
    //finds the angle from center given pixel coordinates              
    double getAngle(float xPixel, float wPixel);     
    //if AR tag found, updates distance, bearing, and id                              
    void updateDetectedTagInfo(rover_msgs::Target *arTags, pair<Tag, Tag> &tagPair, Mat &depth_img, Mat &src); 
    
};
