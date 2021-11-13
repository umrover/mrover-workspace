#pragma once

#include "../artag_detector.hpp"
#include "artag_test_errors.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <experimental/filesystem>
#define IoUThreshold 0.5


class ArtagTest {
public:
    ArtagTest(string & image_, string & filename_);
    float intersectionOverUnion(const vector<cv::Point2f> &predicted_corners, const vector<cv::Point2f> &label_corners);
    void run(TagDetector * detector);
    pair<std::vector<std::vector<cv::Point2f> >, std::vector<int> >  getLabels();
private:
    cv::Mat image;
    string filename;
    string testcase_name;
};

class ArtagTestSuite {
public:
    ArtagTestSuite();
    TagDetector * getDetector();
    void run();
    ~ArtagTestSuite();
private:
    TagDetector * detector;
};