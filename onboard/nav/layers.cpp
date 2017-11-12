#include "layers.hpp"

#include <chrono>

namespace Auton {
    namespace detail {
        GalBlock::GalBlock(System &sys) :
            lock_(sys.global_activity_lock_)
        {
        }
    }

    System::~System() {
        std::unique_lock<std::mutex> lock(global_activity_lock_);
        for (auto it = threads_.begin(); it != threads_.end(); ++it) {
            it->first->active_ = false;
            it->second.join();
            delete it->first;
            it = threads_.erase(it);
        }
    }

    void System::add_layer(Layer *layer) {
        threads_.emplace(layer, std::thread(thread_func, layer));
    }

    detail::GalBlock System::disable_interrupts() {
        return std::move(detail::GalBlock(*this));
    }

    void System::thread_func(Layer *layer) {
        while (layer->active()) {
            layer->wait_while_paused();
            layer->run();
            layer->wait_standard();
        }
    }

    Layer::Layer(System &sys) :
        sys_(sys),
        paused_(true),
        active_(true)
    {
        sys_.add_layer(this);
    }

    Layer::~Layer() {}

    bool Layer::active() {
        std::lock_guard<std::mutex> lock(state_mut_);
        return active_;
    }

    void Layer::pause() {
        std::lock_guard<std::mutex> lock(state_mut_);
        paused_ = true;
    }

    void Layer::resume() {
        std::lock_guard<std::mutex> lock(state_mut_);
        paused_ = false;
        paused_cv_.notify_one();
    }

    void Layer::wait_while_paused() {
        std::unique_lock<std::mutex> lock(state_mut_);
        paused_cv_.wait(lock, [&]() {return !paused_;});
    }

    void Layer::wait_standard() {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(500ms);
    }

    detail::GalBlock Layer::no_interrupt() {
        return std::move(sys_.disable_interrupts());
    }
}
