#include "artag_test_errors.hpp"

CountingError::CountingError(TestError::ErrorString type_, int actual_, int correct_) 
    : type(TestError::ErrorValues[type_]), actual(actual_), correct(correct_) {}

void CountingError::print() {
    std::cout << "Expected " << correct << " " << type << " but actual value was " << actual << "\n";
}

ThresholdError::ThresholdError(TestError::ErrorString type_, float value_, float threshold_)
    : type(TestError::ErrorValues[type_]), value(value_), threshold(threshold_) {}

void ThresholdError::print() {
    std::cout << "Threshold for " << type << " is " << threshold << " but actual value was " << value << "\n";
}

FileError::FileError(std::string filename_) 
    : filename(filename_) {}

void FileError::print() {
    std::cout << "File " <<  filename << " could not be opened\n"; 
}