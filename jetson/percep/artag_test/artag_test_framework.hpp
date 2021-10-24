#pragma once

#include "../artag_detector.hpp"
#include "artag_test_errors.hpp"
#include <vector>
#define IoUThreshold 0.5


class ArtagTest {
public:
    ArtagTest(cv::Mat image_);
    float intersectionOverUnion(const vector<cv::Point2f> &predicted_corners, const vector<cv::Point2f> &label_corners);
    void run()
private:
    cv::Mat image;
};

class ArtagTestSuite {
public:
    ArtagTestSuite()
private:
    TagDetector * detector;
};