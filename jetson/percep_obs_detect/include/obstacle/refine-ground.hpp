#include "plane-ransac.hpp"
#include "euclidean-cluster.hpp"

class RefineGround {
    public:
        float refineDistance = 100.0f, refineHeight = 200.0f;

        // remove obstacles that are shorter than max height and less than distance away from the ground plane
        void pruneObstacles(Plane& groundPlane, EuclideanClusterExtractor::ObsReturn& obstacles);
};
