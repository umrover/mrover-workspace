#pragma once

#include "../artag_detector.hpp"
#include <vector>
#include <iostream>
#define IoUThreshold 0.5

class TestError : public std::exception {
    enum ErrorString {
        corners = "corners",
        artag = "artags",
        id = "to be the tag's id",
        intersection = "intersection over union"
    };

    virtual void print() = 0;
};

class CountingError : public TestError {
    CountingError(TestError::ErrorString type_, int actual_, int correct_);

    void print();

private:
    TestError::ErrorString type;
    int actual;
    int correct;
};

class ThresholdError : public TestError {
    ThresholdError(TestError::ErrorString type_, float value_, float threshold_);

    void print();

private:
    float type_
    float value_
    float threshold_
};

class ArtagTest {
    float intersectionOverUnion(const vector<Point2f> &predicted_corners, const vector<Point2f> &label_corners);
};

class ArtagTestSuite {

};