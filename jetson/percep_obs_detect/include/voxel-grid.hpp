#include "common.hpp"
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>

#pragma once

/** 
 * \class voxelGrid
 * \brief class that generates Voxel Grid of point cloud
 */
class VoxelGrid {
public: 
    
    /**
     * \brief VoxelGrid constructor
     * \param partitions: number of divisions on each axis of voxel grid
     */
    VoxelGrid(int partitions);

    /**
     * \brief given a PC finds the 6 extrema that define a bounding cube. 
     * Then sort the GPU cloud according to how points hash to cube
     * \param pc: GPU point cloud will be modified by in place sort
     * \return structure containing start value of each bin in sorted GPU cloud
     */
    Bins run(GPU_Cloud_F4 &pc);

private:

    int partitions; // Number of divisions made on each axis
    std::pair<float,float>* extremaValsGPU; // Array contianing max and min vals per axis
    enum axis{x=0, y=1, z=2}; // Enum to make referencing axis intuitive
    Bins bins;

};

