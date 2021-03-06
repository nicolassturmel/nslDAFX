//
//  DelayModel.hpp
//  AudioUnitV3Example
//
//  This class performs a double tap time varying delay computation, a filter is inserted in the feedback loop
//
//  Time varying + filter = tape delay simulation
//
//  How to use it :
//      1/ Init class
//      2/ Set parameters
//      3/ Write a callback to handle the buffer (this class only performs delay computation one sample at a time)
//          for each sample in the buffer
//              for each channel
//                      perform delay computation of the sample / channel pair
//              update delay
//      4/ Enjoy !
//
//  To do : pass parameter structure in the constructor
//
//  Created by Nicolas STURMEL on 06/02/2016.
//

#ifndef DelayModel_hpp
#define DelayModel_hpp

namespace nslDAFX {
    
    /*
     * You should set this DEFINE to enable sinc approximation
     * TO DO : Fix the computation, it is BUGGY
     */
    //#define SINC_APPROX
    
    /*
     *  Set this value to define the number of tap for the ASRC
     *  ASRC is computed live and the number of tap can be really heavy on the computing
     *  I recommend a range of 4 (9 taps) leading to a THD of 30dB (more than enough for a tape delay)
     *
     */
#ifndef ASRC_TAP_RANGE
#define ASRC_TAP_RANGE 7
#endif
    
#include <math.h>
#include <stdlib.h>
#include <string.h>
    
    class CDelayModel {
    private:
        // Delay buffer
        float ** mSamples;
        
        // Buffer length (time and channels)
        int mSampleSize;
        int mNumberOfChannels;
        
        // Writing head position
        int * mWrite;
        
        // Delays length (in samples)
        int mDelay;
        int mDelay2;
        
        // Target delays when changed by user
        int mTargetDelay;
        int mTargetDelay2;
        
        // System sampling frequency
        double mSamplingFrequency;
        
        // Delay feedback
        double mFeedback;
        
        // Delay wetness
        double mWetness;
        
        // Delay high frequency cutoff
        double mAlpha;
        
        // Delay wow depth
        double mWow;
        
        // Delay wow frequency
        double mWowFreq;
        
        // Delay offset incured by wow
        double * mDelayOffset;
        
    public:
        
        /*
         *      Initiate buffers for the delay
         *              Size : delay size in samples
         *              Channels : number of channels for the delay
         *              SamplingFrequency : sampling frequency for the effect
         *
         */
        void init(int Size, int Channels, int SamplingFrequency);
        
        /*
         *      Setter for the low-pass filter parameter (0 is no filtering, 1 is maximum filtering)
         * 
         *      returns set value
         */
        double setAlpha(double param);
        
        /*
         *      Setter for the first delay, in seconds
         *
         *      returns set value
         */
        double setDelay(double Time);
        
        /*
         *      Setter for the second delay, in seconds
         *
         *      returns set value
         */
        double setDelay2(double Time);
        
        /*
         *      Setter for the feedback (0 no feedback, 1 maximum feedback)
         *
         *      returns set value
         */
        double setFeedback(double Feedback);
        
        /*
         *      Setter for the wow parameter (0 : now wow, 1 : maximum wow)
         *
         *      returns set value
         */
        double setWow(double wow);
        
        /*
         *      Setter for the wow frequency (0 : 0Hz, 1 : maximum frequency)
         *
         *      returns set value
         */
        double setWowFreq(double wowFreq);
        
        /*
         *      Setter for the dry/wet parameter (0 : dry, 1 : wet)
         *
         *      returns set value
         */
        double setWet(double wet);
        
        /*
         *      Computes delay in place (for each sample) for a given channel
         *
         *      returns computed value
         */
        float delay(int channel, float input);
        
        /*
         *      For each sample, you should update the delay (only necessary if delay line 1 or 2 are controllable)
         */
        void adjustDelay();
        
        
    };
    
}

#endif /* DelayModel_hpp */
