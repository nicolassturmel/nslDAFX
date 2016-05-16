//
//  DelayModel.cpp
//  AudioUnitV3Example
//
//  Created by Nicolas STURMEL on 06/02/2016.
//
//

#include "DelayModel.hpp"

namespace nslDAFX {
    
#ifdef SINC_APPROX
    double * sinc_tab;
    int sinc_step;
    
    void sinc_init(int step)
    {
        sinc_step = step;
        sinc_tab = (double *)malloc(sizeof(double)*sinc_step);
        for(int i = 0; i < sinc_step ; i++)
        {
            sinc_tab[i] = sin(3.14159*((double)i/(double)sinc_step));
        }
    }
    
    static inline double sin_approx(double y)
    {
        double x = (y > 0)? y : -y;
        int a = x;
        double p = x - a;
        int i = p*sinc_step;
        double q = p*sinc_step - i;
        double r = sinc_tab[i] + (sinc_tab[i+1]-sinc_tab[i])*q;
        if(a%2 == 0)
            return r;
        else
            return -r;
    }
    
#endif
    
    double sinc(double x)
    {
        if(x == 0) return 1;
#ifdef SINC_APPROX
        return sin_approx(x)/(M_PI*x);
#else
        return sinf(M_PI*x)/float(M_PI*x);
#endif
    }
    
    
    void CDelayModel::init(int Size, int Channels, int SamplingFrequency)
    {
#ifdef SINC_APPROX
        sinc_init(1000);
#endif
        mSamples = (float **)malloc(Channels*sizeof(float *));
        for(int i = 0; i < Channels; i++)
        {
            mSamples[i] = (float *)malloc(Size*sizeof(float));
            memset(mSamples[i],0,Size*sizeof(float));
        }
        mSampleSize = Size;
        mNumberOfChannels = Channels;
        mDelayOffset = (double *)malloc(Channels*sizeof(double));
        mWrite = (int *)malloc(Channels*sizeof(int));
        for(int i = 0; i < Channels ; i++)
        {
            mWrite[i] = mSampleSize;
            mDelayOffset[i] = 0.f;
        }
        mDelay = 0;
        mTargetDelay = 0;
        mDelay2 = 0;
        mTargetDelay2 = 0;
        mFeedback = 0.f;
        mWetness = 0.5f;
        mSamplingFrequency = SamplingFrequency;
        mAlpha = 0.5;
        mWow = 0.3;
        mWowFreq = 0.3;
    };
    
    double CDelayModel::setAlpha(double param)
    {
        if(param > 0 && param < 1)
        {
            mAlpha = 1-param*param;
        }
        return mAlpha;
    };
    
    double CDelayModel::setDelay(double Time)
    {
        int Del = (Time/1000.f*mSamplingFrequency);
        if(Del > mSampleSize-1) Del = mSampleSize-1;
        
        mTargetDelay = Del;
        
        return mTargetDelay*(1000.f)/mSamplingFrequency;
    };
    
    double CDelayModel::setDelay2(double Time)
    {
        int Del = (Time/1000.f*mSamplingFrequency);
        if(Del > mSampleSize-1) Del = mSampleSize-1;
        
        mTargetDelay2 = Del;
        
        return mTargetDelay2*(1000.f)/mSamplingFrequency;
    }
    
    double CDelayModel::setFeedback(double Feedback)
    {
        if(Feedback > -1 && Feedback < 1) mFeedback = Feedback;
        return mFeedback;
    }
    
    double CDelayModel::setWow(double wow)
    {
        if(wow > 0. && wow < 1.) mWow = wow;
        return mWow;
    }
    
    double CDelayModel::setWowFreq(double wowFreq)
    {
        if(wowFreq > 0. && wowFreq < 1.) mWowFreq = wowFreq;
        return mWowFreq;
    }
    
    double CDelayModel::setWet(double wet)
    {
        if(wet > 0. && wet < 1.) mWetness = wet;
        return mWetness;
    }
    
    float CDelayModel::delay(int channel, float input)
    {
        if(channel > mNumberOfChannels) return 0;
        float output = 0;
        mDelayOffset[channel] = mWow*100.*(1+sin(2.*M_PI*10.*mWowFreq/mSamplingFrequency*mWrite[channel]));
        
        int p = (int)mDelayOffset[channel];
        double frac = (double)p - mDelayOffset[channel];
        double sum = 0;
        for(int i = -ASRC_TAP_RANGE; i < ASRC_TAP_RANGE; i ++)
        {
            float s = sinc((frac + i)*mAlpha)*cosf(M_PI_2*i/float(ASRC_TAP_RANGE+1));
            if(mDelay > 0.00001)
            {
                output += mSamples[channel][(mWrite[channel] - mDelay - p - i)%mSampleSize]*s;
                sum += s;
            }
            if(mDelay2 > 0.00001)
            {
                output += mSamples[channel][(mWrite[channel] - mDelay2 - p - i)%mSampleSize]*s;
                sum += s;
            }
        }
        //output = filter(channel, output)/sqrt(2);
        if(sum > 0.5)
            output = output/sum;
        mSamples[channel][(mWrite[channel])%mSampleSize] = output*mFeedback + input;
        
        output = output*sin(M_PI_2*mWetness)+input*cos(M_PI_2*mWetness);
        mWrite[channel]++;
        
        return output;
    }
    
    void CDelayModel::adjustDelay()
    {
        if(mDelay != mTargetDelay)
        {
            if(mDelay > mTargetDelay) mDelay--;
            else mDelay++;
        }
        
        if(mDelay2 != mTargetDelay2)
        {
            if(mDelay2 > mTargetDelay2) mDelay2--;
            else mDelay2++;
        }
    }
    
}