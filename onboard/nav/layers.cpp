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
            it->first->active_.set(false);
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
        return this->active_.clone();
    }

    void Layer::pause() {
        this->paused_.set(true);
    }

    void Layer::resume() {
        this->paused_.set(false);
    }

    void Layer::wait_while_paused() {
        paused_.wait_for([&](bool p) {return !p;});
    }

    void Layer::wait_standard() {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(500ms);
    }

    detail::GalBlock Layer::no_interrupt() {
        return std::move(sys_.disable_interrupts());
    }
}
