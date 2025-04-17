// SignalFilters.cpp
#include "SignalFilters.h"
#include <math.h>

SignalFilters::SignalFilters(float sampleRate) : _sampleRate(sampleRate) {
}

FilterCoefficients SignalFilters::designLowPassFilter(float cutoffFreq) {
    FilterCoefficients coeffs;
    
    // Normalize frequency
    float omega = 2.0f * PI * cutoffFreq / _sampleRate;
    float alpha = sin(omega) / (2.0f * 0.7071f); // Q factor = 0.7071 for Butterworth
    
    // Calculate coefficients (bilinear transform)
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // Normalized coefficients
    coeffs.b[0] = (1.0f - cosw) / (2.0f * a0);
    coeffs.b[1] = (1.0f - cosw) / a0;
    coeffs.b[2] = (1.0f - cosw) / (2.0f * a0);
    coeffs.a[0] = -2.0f * cosw / a0;
    coeffs.a[1] = (1.0f - alpha) / a0;
    
    return coeffs;
}

FilterCoefficients SignalFilters::designHighPassFilter(float cutoffFreq) {
    FilterCoefficients coeffs;
    
    // Normalize frequency
    float omega = 2.0f * PI * cutoffFreq / _sampleRate;
    float alpha = sin(omega) / (2.0f * 0.7071f); // Q factor = 0.7071 for Butterworth
    
    // Calculate coefficients (bilinear transform)
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // Normalized coefficients
    coeffs.b[0] = (1.0f + cosw) / (2.0f * a0);
    coeffs.b[1] = -(1.0f + cosw) / a0;
    coeffs.b[2] = (1.0f + cosw) / (2.0f * a0);
    coeffs.a[0] = -2.0f * cosw / a0;
    coeffs.a[1] = (1.0f - alpha) / a0;
    
    return coeffs;
}

FilterCoefficients SignalFilters::designBandPassFilter(float lowFreq, float highFreq) {
    FilterCoefficients coeffs;
    
    // Calculate center frequency and bandwidth
    float centerFreq = sqrt(lowFreq * highFreq);
    float bandwidth = highFreq - lowFreq;
    
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float alpha = sin(omega) * sinh(log(2.0f) / 2.0f * bandwidth / centerFreq * omega / sin(omega));
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // Normalized coefficients
    coeffs.b[0] = alpha / a0;
    coeffs.b[1] = 0.0f;
    coeffs.b[2] = -alpha / a0;
    coeffs.a[0] = -2.0f * cosw / a0;
    coeffs.a[1] = (1.0f - alpha) / a0;
    
    return coeffs;
}

FilterCoefficients SignalFilters::designBandStopFilter(float lowFreq, float highFreq) {
    FilterCoefficients coeffs;
    
    // Calculate center frequency and bandwidth
    float centerFreq = sqrt(lowFreq * highFreq);
    float bandwidth = highFreq - lowFreq;
    
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float alpha = sin(omega) * sinh(log(2.0f) / 2.0f * bandwidth / centerFreq * omega / sin(omega));
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // Normalized coefficients
    coeffs.b[0] = 1.0f / a0;
    coeffs.b[1] = -2.0f * cosw / a0;
    coeffs.b[2] = 1.0f / a0;
    coeffs.a[0] = -2.0f * cosw / a0;
    coeffs.a[1] = (1.0f - alpha) / a0;
    
    return coeffs;
}

FilterCoefficients SignalFilters::designNotchFilter(float centerFreq, float Q) {
    FilterCoefficients coeffs;
    
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float alpha = sin(omega) / (2.0f * Q);
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // Normalized coefficients
    coeffs.b[0] = 1.0f / a0;
    coeffs.b[1] = -2.0f * cosw / a0;
    coeffs.b[2] = 1.0f / a0;
    coeffs.a[0] = -2.0f * cosw / a0;
    coeffs.a[1] = (1.0f - alpha) / a0;
    
    return coeffs;
}

float SignalFilters::applyFilter(const float input, FilterCoefficients &coeffs, float *state) {
    // Direct Form II implementation
    float w = input - coeffs.a[0] * state[0] - coeffs.a[1] * state[1];
    float output = coeffs.b[0] * w + coeffs.b[1] * state[0] + coeffs.b[2] * state[1];
    
    // Update state
    state[1] = state[0];
    state[0] = w;
    
    return output;
}