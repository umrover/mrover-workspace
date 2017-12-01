#pragma once

#include <mutex>
#include <condition_variable>

namespace Thor {
    template <typename T>
    class Volatile {
        private:
            class Ref;
            class ConstRef;
        public:
            Volatile() {}
            Volatile(const T & val) : val_(val) {}

            void set(const T & val) {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->val_ = val;
                this->cv_.notify_all();
            }
            void unsafe_set_possibly_race(const T & val) {
                this->val_ = val;
            }

            template <typename Function>
            void wait_for(Function func) {
                std::unique_lock<std::mutex> lock_(this->mut_);
                this->cv_.wait(lock_, [&]() {
                    return func(this->val_);
                });
            }

            T clone() {
                std::unique_lock<std::mutex> lock_(this->mut_);
                return T(this->val_);
            }
            const ConstRef get_ref() const {
                return ConstRef(*this);
            }
            Ref get_mut() {
                return Ref(*this);
            }

        private:
            T val_;
            mutable std::mutex mut_;
            mutable std::condition_variable cv_;

            void notify_all() {
                this->cv_.notify_all();
            }

            class Ref {
                friend class Volatile;
                public:
                    Ref(Volatile & v) 
                        : ref_(v.val_), vol_(v), lock_(v.mut_) {}
                    ~Ref() {
                        this->vol_.notify_all();
                    }

                    T & operator*() {
                        return this->ref_;
                    }
                private:
                    T & ref_;
                    Volatile & vol_;
                    std::unique_lock<std::mutex> lock_;
            };

            class ConstRef {
                public:
                    ConstRef(Volatile & v) 
                        : ref_(v.val_), lock_(v.mut_) {}

                    const T & operator*() {
                        return this->ref_;
                    }
                private:
                    const T & ref_;
                    std::unique_lock<std::mutex> lock_;
            };
    };
}
