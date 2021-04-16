#include <iostream>
#include <sl/Camera.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
using namespace boost::interprocess;
using namespace std;

bool checkStatus(cudaError_t status) {
	if (status != cudaSuccess) {
		printf("%s \n", cudaGetErrorString(status));
		return true;
	}
    return false;
}

int main() {
    // Open camera
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    init_params.camera_resolution = sl::RESOLUTION::VGA; 
    init_params.camera_fps = 100;
    zed.open(init_params); 
    sl::Resolution cloud_res(320/2, 180/2);

    cudaIpcMemHandle_t my_handle;
    sl::Mat operationalFrame, curFrame; 

    // Create shared memory region used for passing the GPU data handle between programs
    shared_memory_object::remove("gpuhandle");
    shared_memory_object shm (create_only, "gpuhandle", read_write);
    shm.truncate(1000);
    mapped_region region(shm, read_write);
    unsigned char *sharedMemData = static_cast<unsigned char*>(region.get_address());
    sharedMemData[sizeof(my_handle)+1] = 0;

    bool gotFrame = false;

    //TEMP, REMOVE
    gotFrame = true;
    zed.grab();
    zed.retrieveMeasure(curFrame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 
    cout << "step bytes " << curFrame.getStepBytes() << endl;
    while(true) {

        
        //Mutex lock and move frame to shared memory 
        if(sharedMemData[sizeof(my_handle)] == 0 && gotFrame) {
            //Set the obs detector's operational frame to the current frame, so that it doesn't get destructed/overwritten
            //operational frame needs to take OWNERSHIP of cur frame's memory
            //curFrame.move(operationalFrame);
            //curFrame = sl::Mat(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);
            sl::Mat::swap(curFrame, operationalFrame);

            sl::float4* dataGPU = operationalFrame.getPtr<sl::float4>(sl::MEM::GPU);
            
            //create the cudaIpcMemHandle_t
            checkStatus(cudaIpcGetMemHandle(&my_handle, dataGPU));
            //convert the cudaIpcMemHandle_t into a primative char array
            unsigned char handle_buffer[sizeof(my_handle)];
            memset(handle_buffer, 0, sizeof(my_handle));
            memcpy(handle_buffer, (unsigned char *)(&my_handle), sizeof(my_handle));
            //copy the primative char array bytes of the handle into shared memory
            memcpy(sharedMemData, handle_buffer, sizeof(my_handle));
            //write parity bit to high
            sharedMemData[sizeof(my_handle)] = 1;
            std::cout << "writing to shared" << std::endl;

            for(int i = 0; i < sizeof(my_handle); i++) {
                cout << (int)handle_buffer[i] << " ";
                //sharedMemData[i] = i;
            }
            cout << endl;

            //TEMP
            gotFrame = false;
        } /*
         else { 
        
            //Read a new frame 
            //curFrame = Mat(...)
            zed.grab();
            zed.retrieveMeasure(curFrame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 
            gotFrame = true;
            //std::cout << "grabbing" << std::endl;
        }*/
    }

    return 0;
}