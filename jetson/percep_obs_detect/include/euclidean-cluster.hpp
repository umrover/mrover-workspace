#pragma once

#include "common.hpp"

/*
How this ultimately should work:
1. Using a kernel, associate each point with a bin representing a voxel of 3D space in a key value pair (see Research/fixed radius NN)
2. Sort the pairs using thrust by bin to increasing coalecsing during NN lookup
3. Construct an undirected graph where each point has an edge to its neighbors via the the vertex method described in Research/euclidean cluster extract 
4. Use connected component labeling to propogate labels through the graph for points that belong to the same cluster see (Research/CCL) 
*/

class EuclideanClusterExtractor {
    public:
        struct ObsReturn {
            int size = 0;
            float* minX;
            float* maxX;
            float* minY;
            float* maxY;
            float* minZ;
            float* maxZ;
            
        };

        int bearingRight;
        int bearingLeft;
        /*
        REQUIRES: 
        - Zed point cloud allocated on GPU
        - Radius for fixed radius NN Search 
        - min size in points [removes clusters that are smaller]
        - max size in points 
        EFFECTS:
        - Computes clusters on point cloud based on Euclidean distances 
        */
        EuclideanClusterExtractor(float tolerance, int minSize, float maxSize, size_t cloudArea, int partitions);
        EuclideanClusterExtractor();

        /*
        EFFECTS:
        - Extracts clusters 
        */
        ObsReturn extractClusters(GPU_Cloud_F4 pc, Bins &bins);

    private:
        //user given model parms
        GPU_Cloud_F4 pc;
        float tolerance;
        int minSize;
        float maxSize;
        
        //internal information
        int* neighborLists; //lists of neighbors for each point inline
        int* listStart; //starting indexes of adjacency lists for each pt (size of max pt cloud)
        int * labels; //labels for each point (size of max pt cloud)
        bool* f1; //frontier array 1 (size of max pt cloud)
        bool* f2; //frontier array 2 (size of max pt cloud)
        bool* stillGoing;

        //Bin creation search data
        float* mins;
        float* maxes;
        int** bins; 
        int* binCount;
        int partitions;
};