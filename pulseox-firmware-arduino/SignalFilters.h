// SignalFilters.h
#ifndef SIGNAL_FILTERS_H
#define SIGNAL_FILTERS_H

#include <Arduino.h>

// Filter coefficients structure
struct FilterCoefficients {
    float b[3];  // Numerator coefficients (b0, b1, b2)
    float a[2];  // Denominator coefficients (a1, a2), a0 is assumed to be 1.0
};

class SignalFilters {
public:
    SignalFilters(float sampleRate);
    
    // Filter design methods
    FilterCoefficients designLowPassFilter(float cutoffFreq);
    FilterCoefficients designHighPassFilter(float cutoffFreq);
    FilterCoefficients designBandPassFilter(float lowFreq, float highFreq);
    FilterCoefficients designBandStopFilter(float lowFreq, float highFreq);
    FilterCoefficients designNotchFilter(float centerFreq, float Q = 30.0);
    
    // Filter application (Direct Form II implementation)
    float applyFilter(float input, const FilterCoefficients &coeffs, float *state);
    
    // Get sample rate
    float getSampleRate() const { return _sampleRate; }
    
private:
    float _sampleRate;
};

#endif // SIGNAL_FILTERS_H