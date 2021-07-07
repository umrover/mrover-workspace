#pragma once
#include "common.hpp"

#ifndef PASS_THROUGH
#define PASS_THROUGH

//Functor predicate to check if a point is within some min and max bounds on a particular axis
class WithinBounds : public IPredicateFunctor {
    
public:
    WithinBounds(float min, float max, char axis) : min(min), max(max), axis(axis) {}

    virtual __host__ __device__ bool operator()(const float4 val) override {
        float test = val.x;
        if(axis == 'z') test = val.z;
        else if(axis == 'y') test = val.y;
        return test > min && test < max;
    }

private:
    float min, max;
    char axis;
};

/** 
 * \class PassThrough
 * \brief Filters out points that are less than a certain coordinate or greater than a certain coordinate on a particular cartesian axis
 */
class PassThrough {

public:

    /**
     * \brief PassThrough constructor 
     * \param axis Either 'x', 'y', or 'z', this is the axis we filter on
     * \param min The minimum allowable coordinate of the point on the selected axis
     * \param max The maximum allowable coordinate of the point on the selected axis
     */
    PassThrough(char axis, float min, float max);

    /**
     * \brief Runs the pass through on the given cloud
     * \param cloud Point cloud to be filtered
     */    
    void run(GPU_Cloud &cloud);

    ~PassThrough();

private:
    WithinBounds* withinBoundsPtr;

};

#endif