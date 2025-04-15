// SignalFilters.h
#ifndef SIGNAL_FILTERS_H
#define SIGNAL_FILTERS_H

#include <Arduino.h>
#include <arm_math.h>  // ARM DSP library for Cortex-M processors

// Filter states structure
struct FilterStates {
    arm_biquad_casd_df1_inst_f32 instance;
    float state[4];  // State buffer for the filter (4 states for a 2nd order filter)
};

class SignalFilters {
public:
    SignalFilters(float sampleRate);
    
    // Create filters
    FilterStates createLowPassFilter(float cutoffFreq);
    FilterStates createHighPassFilter(float cutoffFreq);
    FilterStates createBandPassFilter(float lowFreq, float highFreq);
    FilterStates createBandStopFilter(float lowFreq, float highFreq);
    FilterStates createNotchFilter(float centerFreq, float Q = 30.0f);
    
    // Process samples with the filter
    float processSample(float input, FilterStates &filterStates);
    
    // Update filter parameters
    void updateLowPassFilter(FilterStates &filterStates, float cutoffFreq);
    void updateHighPassFilter(FilterStates &filterStates, float cutoffFreq);
    void updateBandPassFilter(FilterStates &filterStates, float lowFreq, float highFreq);
    
    // Get sample rate
    float getSampleRate() const { return _sampleRate; }
    
private:
    float _sampleRate;
    
    // Internal methods to calculate filter coefficients
    void calculateLowPassCoeffs(float cutoffFreq, float *coeffs);
    void calculateHighPassCoeffs(float cutoffFreq, float *coeffs);
    void calculateBandPassCoeffs(float lowFreq, float highFreq, float *coeffs);
    void calculateBandStopCoeffs(float lowFreq, float highFreq, float *coeffs);
    void calculateNotchCoeffs(float centerFreq, float Q, float *coeffs);
};

#endif // SIGNAL_FILTERS_H