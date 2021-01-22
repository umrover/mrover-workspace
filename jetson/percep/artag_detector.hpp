#pragma once

#include <vector>
#include "perception.hpp"
#include "rover_msgs/Target.hpp"

using namespace std;
using namespace cv;

/**
* \struct Tag
* \brief Tag data type groups a Point2f object that represents location and an int that represents ar tag id
*/
struct Tag {
    Point2f loc;
    int id;
};

/**
* \class TagDetector
* \brief Holds the memeber variables and member functions of the TagDetector data type
*/
class TagDetector {
private:
    //Private Member Variables
    Ptr<cv::aruco::Dictionary> alvarDict;
    Ptr<cv::aruco::DetectorParameters> alvarParams;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Mat rgb;

public:
    /**
     \brief constructor loads alvar dictionary data from file that defines tag bit configurations
     \returns void
    */
    TagDetector(); 
    /**
     \brief takes detected AR tag and finds center coordinate for use with ZED
     \param corners: vetor of Point2f objects containing the average location of the 4 corners of the passed-in tag as 
     \returns avgCoord, average location of the 4 corners
    */                                                                 
    Point2f getAverageTagCoordinateFromCorners(const vector<Point2f> &corners);
    /**
     \brief detects AR tags in a given Mat
     \param src: Matrix represtenting source of image
     \param depthsrc: Matrix representing the depth source of the image
     \param rgb: Matrix representing the rbg values of the image
     \returns discoveredTags, pair of tag objects
    */      
    pair<Tag, Tag> findARTags(Mat &src, Mat &depthsrc, Mat &rgb);    
    /**
     \brief finds the angle from center given pixel coordinates
     \param xPixel: x pixel coordinate
     \param wPixel: w pixel coordinate
     \returns angle from the center
    */               
    double getAngle(float xPixel, float wPixel);      
    /**
     \brief if AR tag found, updates distance, bearing, and id  
     \param arTags: pointer to a Target object
     \param tagPair: pair of Tag objects
     \param depthImg: Matrix representing the depth images
     \param src: Matrix represtenting source of image
     \returns void
    */                            
    void updateDetectedTagInfo(rover_msgs::Target *arTags, pair<Tag, Tag> &tagPair, Mat &depthImg, Mat &src); 
};
