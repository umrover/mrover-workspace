#pragma once

#include <array>
#include <numeric>
#include <algorithm>

/***
 * A filter that combines multiple readings into one.
 * A user defined proportion acts as a median filter that gets rids of outliers,
 * which are then piped into a mean filter that averages out the values.
 *
 * @tparam T Reading type
 */
template<typename T>
class Filter {
private:
    std::vector<T> mValues;
    std::vector<T> mSortedValues;
    // After sorting, what proportion in the middle values should we use.
    double mProportion;
    // How many readings we have.
    // This will be capped at the capacity, since when we add when full we will overwrite the oldest value.
    size_t mFilterCount{};
    // Index to the current head.
    // Note this is a circular buffer, so this will wrap around when we reach the end of the internal vector.
    size_t mHead{};

public:
    Filter(size_t size, double centerProportion) : mValues(size), mSortedValues(size), mProportion(centerProportion) {}

    /**
     * Add a value to the filter, overwrites old values if full.
     */
    void push(T value) {
        mHead = (mHead + 1) % size();
        mValues[mHead] = value;
        mFilterCount = std::min(mFilterCount + 1, size());
        mSortedValues.assign(mValues.begin(), mValues.end());
        std::sort(mSortedValues.begin(), mSortedValues.end());
    }

    void reset() {
        mFilterCount = 0;
    }

    //if we have values decrease count by 1
    void decrementCount() {
        mFilterCount = std::max(mFilterCount - 1, size_t{});
    }

    [[nodiscard]] size_t size() const {
        return mValues.size();
    }

    [[nodiscard]] size_t filterCount() const {
        return mFilterCount;
    }

    /***
     * @return If we have enough readings to use the filter
     */
    [[nodiscard]] bool ready() const {
        return mFilterCount > 0;
    }

    [[nodiscard]] bool full() const {
        return mFilterCount == size();
    }

    /***
     * @return Filtered reading if full, or else the most recent reading if we don't have enough readings yet.
     */
    [[nodiscard]] T get() const {
//        return mValues[mHead];
        if (!full()) {
            return mValues[mHead];
        }
        auto begin = mSortedValues.begin() + (mProportion * size() / 4);
        auto end = mSortedValues.end() - (mProportion * size() / 4);
        return std::accumulate(begin, end, T{}) / (end - begin);
    }
};
