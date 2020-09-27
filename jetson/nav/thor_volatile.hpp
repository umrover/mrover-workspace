#pragma once

#include <mutex>
#include <condition_variable>

namespace Thor {
    template <typename T>
    class Volatile {
        public:
            Volatile() : changed_(false) {}
            Volatile(const T & val) : val_(val), changed_(false) {}

            void set(const T & val) {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->unsafe_set_possibly_race(val);
            }
            template <typename Function>
            bool set_conditionally(const T & val, Function func) {
                std::unique_lock<std::mutex> lock_(this->mut_);
                if (func(this->val_)) {
                    this->unsafe_set_possibly_race(val);
                    return true;
                }
                return false;
            }
            void unsafe_set_possibly_race(const T & val) {
                // this->changed_ = (val != this->val_);
                this->val_ = val;
                this->changed_ = true;
                this->cv_.notify_all();
            }

            template <typename Function>
            void wait_for(Function func) {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->cv_.wait(lock_, [&]() {
                    return func(this->val_);
                });
            }

            template <typename Function>
            void transaction(Function func) {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->changed_ |= func(this->val_);
                if (this->changed_) {
                    this->cv_.notify_all();
                }
            }

            T clone_when_changed() const {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->cv_.wait(lock_, [&]() {
                    return this->changed_;
                });
                this->changed_ = false;
                return T(this->val_);
            }

            T clone() const {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->changed_ = false;
                return T(this->val_);
            }

            template <typename Function>
            bool clone_conditional(Function pred, T *t) const {
                std::unique_lock<std::mutex> lock_(this->mut_);
                if (pred(this->val_)) {
                    *t = T(this->val_);
                    this->changed_ = false;
                    return true;
                }
                return false;
            }

        private:
            T val_;
            mutable bool changed_;
            mutable std::mutex mut_;
            mutable std::condition_variable cv_;
    };
}
