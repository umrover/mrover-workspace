#include "pass-through.hpp"
#include <stdlib.h>
#include <cmath>
#include <limits>
#include "common.hpp"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/fill.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/sequence.h>


PassThrough::PassThrough(char axis, float min, float max) : min{min}, max{max}, axis(axis){};


//Functor predicate to check if a point is within some min and max bounds on a particular axis
class WithinBounds {
    public:
        WithinBounds(float min, float max, char axis) : min(min), max(max), axis(axis) {}

        __host__ __device__ bool operator()(const sl::float4 val) {
            float test;
            if(axis == 'z') test = val.z;
            else if(axis == 'y') test = val.y;
            return test > min && test < max;
        }

    private:
        float min, max;
        char axis;
};

//Execute pass through
void PassThrough::run(GPU_Cloud_F4 &cloud){
    if(cloud.size == 0) return;
    
    //Instansiate a predicate functor and copy the contents of the cloud
    WithinBounds pred(min, max, axis);
    thrust::device_vector<sl::float4> buffer(cloud.data, cloud.data+cloud.size);

    //Copy from the temp buffer back into the cloud only the points that pass the predicate 
    sl::float4* end = thrust::copy_if(thrust::device, buffer.begin(), buffer.end(), cloud.data, pred);

    //Clear the remainder of the cloud of points that failed pass through
    thrust::fill(thrust::device, end, cloud.data+cloud.size, sl::float4(0, 0, 0, 0));

    //update the cloud size
    cloud.size = end - cloud.data;
}

