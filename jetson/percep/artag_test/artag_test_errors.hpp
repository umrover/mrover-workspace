#pragma once

#include <string>
#include <iostream>

class TestError : public std::exception {
public:
    virtual void print() = 0;
protected:
    enum ErrorString {
        corners = "corners",
        artag = "artags",
        id = "to be the tag's id",
        intersection = "intersection over union"
    };
};

class CountingError : public TestError {
public:
    CountingError(TestError::ErrorString type_, int actual_, int correct_);

    void print() override;

private:
    TestError::ErrorString type;
    int actual;
    int correct;
};

class ThresholdError : public TestError {
public:
    ThresholdError(TestError::ErrorString type_, float value_, float threshold_);

    void print() override;

private:
    TestError::ErrorString type;
    float value;
    float threshold;
};

class FileError : public TestError {
    FileError(std::string filename_);

    void print() override;
    
private:
    std::string filename;        
};