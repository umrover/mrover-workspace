#pragma once

#include <iostream>

/***
 * Readings from the sensor may occasionally have invalid readings.
 * A cache attempts to resolve this by saving old values for a bit
 * and also not adding new readings until they show up repeatedly.
 *
 * @tparam T Reading type
 */
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

    void reset(){
        mCurrentReading = mInvalidDefault;
        mValid = false;
        mHits = 0;
        mMisses = 0;
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