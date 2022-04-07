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
    // After sorting, what proportion in the middle values should we use.
    double mProportion;

public:
    Filter(size_t size, double centerProportion) : mValues(size), mProportion(centerProportion), mSortedValues(size) {}

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

    [[nodiscard]] size_t size() const {
        return mValues.size();
    }

    /***
     * @return If we have enough readings to use the filter
     */
    [[nodiscard]] bool full() const {
        return mCount == size();
    }

    /***
     * @return Filtered reading if full, or else the most recent reading if we don't have enough readings yet.
     */
    T get() {
        if (!full()) {
            return mValues[mHead];
        }
        mSortedValues = mValues;
        std::sort(mSortedValues.begin(), mSortedValues.end());
        auto begin = mSortedValues.begin() + (mProportion * size() / 4);
        auto end = mSortedValues.end() - (mProportion * size() / 4);
        return std::accumulate(begin, end, T{}) / (end - begin);
    }
};
