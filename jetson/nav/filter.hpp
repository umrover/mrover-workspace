#pragma once

#include <array>
#include <numeric>
#include <algorithm>

template<typename T>
class Filter {
private:
    std::vector<T> mValues;
    std::vector<T> mSortedValues;
    // How many readings we have.
    // This will be capped at the capacity, since when we add when full we will overwrite the oldest value.
    size_t mCount{};
    // Index to the current head.
    // Note this is a circular buffer, so this will wrap around when we reach the end of the internal vector.
    size_t mHead{};

public:
    explicit Filter(size_t size) : mValues(size), mSortedValues(size) {}

    /**
     * Add a value to the filter, overwrites old values if full.
     */
    void push(T value) {
        mHead = (mHead + 1) % size();
        mValues[mHead] = value;
        mCount = std::min(mCount + 1, size());
    }

    void reset() {
        mCount = 0;
    }

    size_t size() {
        return mValues.size();
    }

    /***
     * @return If we have enough readings to use the filter
     */
    bool full() {
        return mCount == size();
    }

    /***
     * @param percentMiddle After sorting, what percent in the middle values should we use.
     * @return Filtered reading if full, or else the most recent reading if we don't have enough readings yet.
     */
    T get(double percentMiddle) {
        if (!full()) {
            return mValues[mHead];
        }
        mSortedValues = mValues;
        std::sort(mSortedValues.begin(), mSortedValues.end());
        auto begin = mSortedValues.begin() + (percentMiddle * size() / 4);
        auto end = mSortedValues.end() - (percentMiddle * size() / 4);
        return std::accumulate(begin, end, T{}) / (end - begin);
    }
};
