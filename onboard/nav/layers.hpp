#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

#include <unordered_map>

namespace Auton {
    class System;
    class Layer;

    namespace detail {
        class GalBlock {
            public:
                GalBlock(Auton::System &sys);

            private:
                std::unique_lock<std::mutex> lock_;
        };
    }

    class System {
        friend class detail::GalBlock;
        public:
            ~System();
            detail::GalBlock disable_interrupts();

            void add_layer(Layer *layer);
        private:
            static void thread_func(Layer *layer);

            std::mutex global_activity_lock_;
            std::unordered_map<Layer *, std::thread> threads_;
    };

    class Layer {
        friend class System;
        public:
            Layer(System &sys);
            virtual ~Layer();

            virtual void run() = 0;

            bool active();

            void pause();
            void resume();
        protected:
            detail::GalBlock no_interrupt();

            void wait_while_paused();
            void wait_standard();
        private:
            System &sys_;

            bool paused_;
            bool active_;

            std::mutex state_mut_;
            std::condition_variable paused_cv_;
    };
}
