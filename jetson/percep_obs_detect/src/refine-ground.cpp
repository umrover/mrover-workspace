#include "refine-ground.hpp"

void RefineGround::pruneObstacles(Plane &groundPlane, EuclideanClusterExtractor::ObsReturn &obstacles) {
    for(int i = 0; i < obstacles.obs.size(); i++) {
        if(obstacles.obs[i].minX < obstacles.obs[i].maxX && obstacles.obs[i].minZ < obstacles.obs[i].maxZ){
            float centerX = (obstacles.obs[i].minX + obstacles.obs[i].maxX)/2;
            float centerY = (obstacles.obs[i].minY + obstacles.obs[i].maxY)/2;
            float centerZ = (obstacles.obs[i].minZ + obstacles.obs[i].maxZ)/2;

            float height = obstacles.obs[i].maxY - obstacles.obs[i].minY;

            float planeY = (-groundPlane.normal.x * centerX + groundPlane.normal.x * groundPlane.p0.x +
                            groundPlane.normal.y * groundPlane.p0.y +
                            -groundPlane.normal.z * centerZ + groundPlane.normal.z * groundPlane.p0.z)/groundPlane.normal.y;

            if(abs(planeY - centerY) < refineDistance && height < refineHeight) {
                obstacles.obs[i].minX = 1;
                obstacles.obs[i].maxX = -1;
                obstacles.obs[i].minZ = 1;
                obstacles.obs[i].maxZ = -1;
            }
        }
    }
}