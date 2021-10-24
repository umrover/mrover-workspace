#pragma once

#include "../artag_detector.hpp"
#include "artag_test_errors.hpp"
#include <vector>
#include <iostream>
#define IoUThreshold 0.5


class ArtagTest {
public:
    ArtagTest(cv::Mat image_);
    float intersectionOverUnion(const vector<cv::Point2f> &predicted_corners, const vector<cv::Point2f> &label_corners);
    void run(TagDetector * detector);
private:
    cv::Mat image;
};

class ArtagTestSuite {
public:
    ArtagTestSuite();
private:
    TagDetector * detector;
};