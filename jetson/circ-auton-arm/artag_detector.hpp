#pragma once

#include <vector>
#include "perception.hpp"
#include "rover_msgs/TargetPosition.hpp"

using namespace std;
using namespace cv;

struct Tag {
    //original location of tag according to opencv
    Point2f loc;
    //location converted to m and on coordinate grid
    Point2f locM;
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
    //constructor loads alvar dictionary data from file that defines tag bit configurations
    TagDetector(); 
    //takes detected AR tag and finds center coordinate for use with ZED                                                                 
    Point2f getAverageTagCoordinateFromCorners(const vector<Point2f> &corners);
    //detects AR tags in a given Mat     
    pair<Tag, Tag> findARTags(Mat &src, Mat &depth_src, Mat &rgb);    
    //finds the angle from center given pixel coordinates              
    double getAngle(float xPixel, float wPixel);     
    //if AR tag found, updates distance, bearing, and id                              
    void updateDetectedTagInfo(rover_msgs::TargetPosition *arTags, pair<Tag, Tag> &tagPair, Mat &depth_img, Mat &src, Mat &rgb); 
    //finds coordinate of tag center relative to frame center
    Point2f getTagCoordRelativeToCenter(const vector<Point2f> &corners, Mat &src, Mat &rgb, Point2f &tagLoc);
};
