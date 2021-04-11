#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

int main(int argc, char *argv[]) {
   
   int arr[3] = {5, 2, 7};

   using namespace boost::interprocess;
   shared_memory_object::remove("MySharedMemory");

   //Create a shared memory object.
   shared_memory_object shm (create_only, "MySharedMemory", read_write);

   //Set size
   shm.truncate(1000);

   //Map the whole shared memory in this process
   mapped_region region(shm, read_write);

   for(int i = 0; i < 3; ++i) {
      std::memcpy(static_cast<int*>(region.get_address())+i, arr+i, region.get_size());
   }
   return 0;
}