#include "refine-ground.hpp"

void RefineGround::pruneObstacles(Plane& groundPlane, EuclideanClusterExtractor::ObsReturn& obstacles) {
    for (EuclideanClusterExtractor::Obstacle& ob: obstacles.obs) {
        if (ob.minX < ob.maxX && ob.minZ < ob.maxZ) {
            float centerX = (ob.minX + ob.maxX) / 2;
            float centerY = (ob.minY + ob.maxY) / 2;
            float centerZ = (ob.minZ + ob.maxZ) / 2;

            float height = ob.maxY - ob.minY;

            float planeY = (-groundPlane.normal.x * centerX + groundPlane.normal.x * groundPlane.p0.x +
                            groundPlane.normal.y * groundPlane.p0.y +
                            -groundPlane.normal.z * centerZ + groundPlane.normal.z * groundPlane.p0.z) / groundPlane.normal.y;

            if (abs(planeY - centerY) < refineDistance && height < refineHeight) {
                ob.minX = 1;
                ob.maxX = -1;
                ob.minZ = 1;
                ob.maxZ = -1;
            }
        }
    }
}