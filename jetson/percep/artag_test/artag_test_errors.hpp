#pragma once

#include <string>
#include <iostream>

class TestError : public std::exception {
public:
    virtual void print() = 0;
protected:
    enum ErrorString {
        corners = 0,
        artag = 1,
        id = 2,
        intersection = 3
    };

    const std::string ErrorValues[4] = {
        "corners", 
        "artags", 
        "to be the tag's id", 
        "intersection over union"
    };

};

class CountingError : public TestError {
public:
    CountingError(TestError::ErrorString type_, int actual_, int correct_);

    void print() override;

private:
    std::string type;
    int actual;
    int correct;
};

class ThresholdError : public TestError {
public:
    ThresholdError(TestError::ErrorString type_, float value_, float threshold_);

    void print() override;

private:
    std::string type;
    float value;
    float threshold;
};

class FileError : public TestError {
    FileError(std::string filename_);

    void print() override;
    
private:
    std::string filename;        
};