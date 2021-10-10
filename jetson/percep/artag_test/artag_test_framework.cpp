#include "artag_test_framework.hpp"

CountingError::CountingError(TestError::ErrorString type_, int actual_, int correct_) 
    : type(type_), actual(actual_), correct(correct_) {}

void CountingError::print() {
    std::cout << "Expected " << correct << " " << type << " but actual value was " << actual << "\n";
}

ThresholdError::ThresholdError(TestError::ErrorString type_, float value_, float threshold_)
    : type(type_), value(value_), threshold(threshold_) {}

void ThresholdError::print() {
    std::cout << "Threshold for " << type << " is " << threshold << " but actual value was " << value << "\n";
}

float ArtagTest::intersectionOverUnion(const vector<Point2f> &predicted_corners, const vector<Point2f> &label_corners)

