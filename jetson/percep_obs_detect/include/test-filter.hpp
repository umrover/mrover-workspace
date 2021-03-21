#include <sl/Camera.hpp>
#include "common.hpp"


#ifndef TEST_FILTER
#define TEST_FILTER


class TestFilter {

    public:
        //Initialized filter with point cloud
        TestFilter();
        
        //Run the filter
        void run(GPU_Cloud_F4 pc);

    

};

#endif