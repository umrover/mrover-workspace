#include "pass-through.hpp"

// PassThrough Constructor
PassThrough::PassThrough(char axis, float min, float max) :
    withinBoundsPtr{new WithinBounds(min, max, axis)} {}

//Execute pass through
void PassThrough::run(GPU_Cloud &cloud){
    Filter<WithinBounds> (cloud, *withinBoundsPtr, FilterOp::REMOVE, 3.57331108403e-43);
}

// PassThrough Destructor
PassThrough::~PassThrough() {
    delete withinBoundsPtr;
}
