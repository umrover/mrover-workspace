#pragma once

#include "common.hpp"
#include <vector>

/**
* \class EuclideanClusterExtractor
* \brief Finds distinct clusters in the point cloud and identifies them
*/
class EuclideanClusterExtractor {
    public:
        /*
        * Obstacle struct, will fill the vector in ObsReturn
        */
        struct Obstacle {
            float minX;
            float maxX;
            float minY;
            float maxY;
            float minZ;
            float maxZ;
        };
        /**
         * \struct ObsReturn
         * \brief Basic datatype to return found clusters
         */
        struct ObsReturn {
            int size = 0;
            std::vector<Obstacle> obs;
        };

        float tolerance;
        int minSize;
        float maxSize;

        int bearingRight;
        int bearingLeft;

        /**
         * \brief EuclideanClusterExtractor constructor
         * \param tolerance How far one point can be from another and still be considered "connected", or part of the same cluster
         * \param minSize The minimum number of points that can define a unique cluster (helpful for noise removal)
         * \param maxSize The maximum number of points that can define a unique cluster (helpful for not identifying walls)
         * \param cloudArea The maximum size of a cloud that may be passed to the extractor
         * \param partitions TODO
         */
        EuclideanClusterExtractor(float tolerance, int minSize, float maxSize, size_t cloudArea, int partitions);

        EuclideanClusterExtractor();

        /**
         * \brief Extracts clusters and packages them into ObsReturn
         * \param pc Point cloud to search
         * \param bins Bins which sub-divide space from VoxelFilter
         */
        ObsReturn extractClusters(GPU_Cloud& pc, Bins& bins);

    private:
        //user given model parms
        GPU_Cloud pc;

        //internal information
        int* neighborLists; //lists of neighbors for each point inline
        int* listStart; //starting indexes of adjacency lists for each pt (size of max pt cloud)
        int* labels; //labels for each point (size of max pt cloud)
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