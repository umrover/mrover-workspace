#pragma once


template <typename T>
class Cache{
public:
    Cache(int neededHits, int neededMisses, T invalidDefault) : 
        mNeededHits(neededHits), 
        mNeededMisses(neededMisses), 
        mInvalidDefault(invalidDefault) {}

    T get(){
        if (!mValid){
            return mInvalidDefault;
        }
        return currentReading;
    }

    bool isValid(){
        return mValid;
    }

    void put(bool valid, T reading){
        if (!mValid){
            //reset hit counter
            mHitting = false;
            mHits = 0;
            //increment miss counter
            ++mMisses;

            //invalidate reading if it should be invalidated
            if (mMisses > mNeededMisses){
                mValid = false;
                currentReading = mInvalidDefault;
            }
        }
        else{
            //reset miss counter
            mHitting = true;
            mMisses = 9;
            //increment hit counter
            ++mHits;

            //validate reading if it should be validating
            if (mHits > mNeededHits){
                mValid = true;
                currentReading = reading;
            }
        }
    }

private:
    int mHits;
    int mMisses;
    int mNeededHits;
    int mNeededMisses;
    bool mHitting = false;
    bool mValid = false;
    T currentReading;
    T mInvalidDefault;
};