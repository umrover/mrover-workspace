#pragma once

#include <array>
#include <numeric>
#include <algorithm>

template<typename T>
class Filter {
private:
    std::vector<T> mValues;
    std::vector<T> mSortedValues;
    size_t mCount{};
    size_t mHead{};

public:
    explicit Filter(size_t size) : mValues(size), mSortedValues(size) {}

    void add(T value) {
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

    bool get(double percentMiddle, T& out) {
        if (mCount < size()) {
            return false;
        }

        mSortedValues = mValues;
        std::sort(mSortedValues.begin(), mSortedValues.end());
        auto begin = mSortedValues.begin() + (percentMiddle * size() / 4);
        auto end = mSortedValues.end() - (percentMiddle * size() / 4);
        out = std::accumulate(begin, end) / (end - begin);
        return true;
    }
};