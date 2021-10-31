#pragma once

#include "../artag_detector.hpp"
#include "artag_test_errors.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#define IoUThreshold 0.5


class ArtagTest {
public:
    ArtagTest(string & image_, string & filename_, string & testcase_name_);
    float intersectionOverUnion(const vector<cv::Point2f> &predicted_corners, const vector<cv::Point2f> &label_corners);
    void run(TagDetector * detector);
    std::vector<pair<std::vector<cv::Point2f>, int> > getLabels();
    //std::vector<pair<
    //    pair<std::vector<cv::Point2f>, int>, 
    //    pair<std::vector<cv::Point2f>, int> > > 
    //    matchPredictionsAndLabels();
private:
    cv::Mat image;
    string filename;
    string testcase_name;
};

class ArtagTestSuite {
public:
    ArtagTestSuite();
    void run();
private:
    TagDetector * detector;
};