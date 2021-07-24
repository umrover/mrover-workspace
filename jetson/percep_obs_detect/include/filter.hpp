#include "common.hpp"

class IPredicateFunctor {
public:
    virtual __host__ __device__ bool operator()(const float4 val) = 0;
};

enum class FilterOp {REMOVE, COLOR};

template<typename T>
void Filter(GPU_Cloud &cloud, T &pred, FilterOp operation, float color);

//Functor predicate to check if a point is within some min and max bounds on a particular axis
class WithinBounds : public IPredicateFunctor {
public:
    WithinBounds(float min, float max, char axis) : min(min), max(max), axis(axis) {}

    virtual __host__ __device__ bool operator()(const float4 val) override {
        float test = val.x;
        if(axis == 'z') test = val.z;
        else if(axis == 'y') test = val.y;
        return min < test && test < max;
    }

private:
    float min, max;
    char axis;
};

// Functor predicate to check if a point is considered within the plane
class NotInPlane : public IPredicateFunctor {
public:
    __host__ __device__ NotInPlane(float3 planeNormal, float3 ptInPlane, int threshold) : planeNormal{ normalize(planeNormal) }, ptInPlane { ptInPlane}, threshold{ threshold } {}

    virtual __host__ __device__ bool operator()(const float4 val) override {
        
        // Compute distsance point is from plane
        float3 curPt = make_float3(val.x, val.y, val.z);
        float3 d_to_model_pt = curPt - ptInPlane;
        float d = abs(dot(planeNormal, d_to_model_pt));

        // We don't want to copy anything in the plane
        return (d < threshold) ? false : true;
    }

private:
    float3 planeNormal;
    float3 ptInPlane;
    int threshold;
};