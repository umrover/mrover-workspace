#pragma once

#include "common.hpp"

/**
 * \class IPredicateFunctor
 * \brief Interface for the predicate functor that is passed to the Filter class
 */
class IPredicateFunctor {
    public:

        /**
         * \brief abstract operator that needs to be overloaded for the filter predicate to be valid
         * \param val: point that is assessed by the predicate to be filtered or not filtered
         * \return true if point should be untouched, false if the point will be colored or removed
         */
        virtual __host__ __device__ bool operator()(const float4 val) = 0;
};

/**
 * \enum specifies the filter operation of removing or coloring points
 */
enum class FilterOp {
    REMOVE, COLOR
};

/**
 * \class Filter
 * \brief filters out points in the given cloud based on the given predicate
 * \param cloud: point cloud that will be filtered
 * \param pred: predicate that will be used to evaluate the points that need to be filtered
 * \param operation: enum specifying whether points should be colored or removed
 * \param color: the color that the points will be colored. Unused if operation is remove
 */
template<typename T>
void Filter(GPU_Cloud& cloud, T& pred, FilterOp operation, float color);

/**
 * \class WithinBounds
 * \brief Functor used by pass-through for filtering out all points not within a given range
 */
class WithinBounds : public IPredicateFunctor {
    public:
        /**
         * \brief constructor
         * \param min: lower bound that point must be greater than
         * \param max: upper bound that point must be less than
         * \param axis: axis that value being used for comparison should be checked
         */
        WithinBounds(float min, float max, char axis) : min{min}, max{max}, axis{axis} {}

        /**
         * \brief operator called to compare points in the cloud against
         * \param val: point that is assessed by the predicate to be filtered or not filtered
         * \return true if point the axis of the point is within the bounds, false otherwise
         */
        virtual __host__ __device__ bool operator()(const float4 val) override {
            float test = val.x;
            if (axis == 'z') test = val.z;
            else if (axis == 'y') test = val.y;
            return min < test && test < max;
        }

    private:
        float min, max;
        char axis;
};

/**
 * \class NotInPlane
 * \brief Functor used by plane-ransac to determine whether a point is in the plane or not in the plane
 */
class NotInPlane : public IPredicateFunctor {
    public:

        /**
         * \brief constructor
         * \param planeNormal: vector normal to the plane
         * \param ptInPlane: a point that lies in the plane
         * \param threshold: distance from the plane a point can be to be considered 'in' the plane
         */
        __host__ __device__ NotInPlane(float3 planeNormal, float3 ptInPlane, int threshold) : planeNormal{normalize(planeNormal)},
                                                                                              ptInPlane{ptInPlane}, threshold{threshold} {}

        /**
         * \brief operator called to compare points in the cloud against
         * \param val: point that is assessed by the predicate to be filtered or not filtered
         * \return true if point is not in the plane, false if point is in the plane
         */
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
