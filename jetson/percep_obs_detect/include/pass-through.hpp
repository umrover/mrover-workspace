#pragma once
#include <sl/Camera.hpp>
#include "common.hpp"


#ifndef PASS_THROUGH
#define PASS_THROUGH

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
        void run(GPU_Cloud_F4 &cloud);

    private:

        float min;
        float max;

        char axis;

};

#endif