#pragma once
#include <sl/Camera.hpp>
#include "common.hpp"


#ifndef PASS_THROUGH
#define PASS_THROUGH

class PassThrough {

    public:
    PassThrough();

    //Constructor takes in axis, min, max
    PassThrough(char axis, float min, float max);

    //Main processing function
    void run(GPU_Cloud_F4 &);

    private:

    float min;
    float max;

    char axis;

};

#endif