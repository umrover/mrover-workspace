#pragma once

#include <iostream>

template<typename T>
class Cache {
public:
    Cache(int neededHits, int neededMisses, T invalidDefault) :
            mNeededHits(neededHits),
            mNeededMisses(neededMisses),
            mInvalidDefault(invalidDefault) {}

    T get() const {
        return mValid ? mCurrentReading : mInvalidDefault;
    }

    bool isValid() {
        return mValid;
    }

    void put(bool currentReadingValid, T reading) {
        if (currentReadingValid) {
            // reset miss counter
            mMisses = 0;
            // increment hit counter
            ++mHits;

            // validate reading if it should be validating
            if (mHits > mNeededHits) {
                mValid = true;
                mCurrentReading = reading;
            }
        } else {
            // reset hit counter
            mHits = 0;
            // increment miss counter
            ++mMisses;

            // invalidate reading if it should be invalidated
            if (mMisses > mNeededMisses) {
                mValid = false;
                mCurrentReading = mInvalidDefault;
            }
        }
    }

private:
    int mHits = 0;
    int mMisses = 0;
    int mNeededHits;
    int mNeededMisses;
    bool mValid = false;
    T mCurrentReading;
    T mInvalidDefault;
};