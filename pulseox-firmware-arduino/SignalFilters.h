// SignalFilters.h
#ifndef SIGNAL_FILTERS_H
#define SIGNAL_FILTERS_H

#include <Arduino.h>
#include <math.h>

// Define PI with fewer decimal points for better numerical stability
#define FILTER_PI 3.14159f

// Simple biquad filter class
class SignalFilters {
public:
    // Simple biquad filter state
    struct FilterState {
        float x1 = 0.0f;  // Previous input
        float x2 = 0.0f;  // Input from 2 samples ago
        float y1 = 0.0f;  // Previous output
        float y2 = 0.0f;  // Output from 2 samples ago
    };
    
    // Biquad filter coefficients
    struct BiquadCoefficients {
        float b0 = 1.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;
    };
    
    SignalFilters(float sampleRate);
    
    // Filter design methods
    BiquadCoefficients createLowPassFilter(float cutoffFreq);
    BiquadCoefficients createHighPassFilter(float cutoffFreq);
    BiquadCoefficients createBandPassFilter(float lowFreq, float highFreq);
    BiquadCoefficients createBandStopFilter(float lowFreq, float highFreq);
    BiquadCoefficients createNotchFilter(float centerFreq, float Q = 30.0f);
    
    // Apply filter to a single sample
    float applyFilter(float input, const BiquadCoefficients &coeffs, FilterState &state);
    
    // Simple DC removal filter
    float applyDCRemoval(float input, float &prevInput, float &prevOutput, float alpha = 0.995f);
    
    // Get sample rate
    float getSampleRate() const { return _sampleRate; }
    
private:
    float _sampleRate;
};

#endif // SIGNAL_FILTERS_H