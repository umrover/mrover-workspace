#include "pass-through.hpp"

// PassThrough Constructor
PassThrough::PassThrough(char axis, float min, float max) :
    withinBoundsPtr{new WithinBounds(min, max, axis)} {}

//Execute pass through
void PassThrough::run(GPU_Cloud &cloud){
    filter<WithinBounds> (cloud, *withinBoundsPtr);
}

// PassThrough Destructor
PassThrough::~PassThrough() {
    delete withinBoundsPtr;
}
