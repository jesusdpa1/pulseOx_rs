// SignalFilters.cpp
#include "SignalFilters.h"
#include <math.h>

SignalFilters::SignalFilters(float sampleRate) : _sampleRate(sampleRate) {
}

FilterStates SignalFilters::createLowPassFilter(float cutoffFreq) {
    FilterStates filterStates;
    
    // Initialize state buffer
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Calculate coefficients (5 values: b0, b1, b2, a1, a2)
    float coeffs[5];
    calculateLowPassCoeffs(cutoffFreq, coeffs);
    
    // Initialize ARM DSP biquad filter structure
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
    
    return filterStates;
}

FilterStates SignalFilters::createHighPassFilter(float cutoffFreq) {
    FilterStates filterStates;
    
    // Initialize state buffer
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Calculate coefficients
    float coeffs[5];
    calculateHighPassCoeffs(cutoffFreq, coeffs);
    
    // Initialize ARM DSP biquad filter structure
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
    
    return filterStates;
}

FilterStates SignalFilters::createBandPassFilter(float lowFreq, float highFreq) {
    FilterStates filterStates;
    
    // Initialize state buffer
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Calculate coefficients
    float coeffs[5];
    calculateBandPassCoeffs(lowFreq, highFreq, coeffs);
    
    // Initialize ARM DSP biquad filter structure
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
    
    return filterStates;
}

FilterStates SignalFilters::createBandStopFilter(float lowFreq, float highFreq) {
    FilterStates filterStates;
    
    // Initialize state buffer
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Calculate coefficients
    float coeffs[5];
    calculateBandStopCoeffs(lowFreq, highFreq, coeffs);
    
    // Initialize ARM DSP biquad filter structure
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
    
    return filterStates;
}

FilterStates SignalFilters::createNotchFilter(float centerFreq, float Q) {
    FilterStates filterStates;
    
    // Initialize state buffer
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Calculate coefficients
    float coeffs[5];
    calculateNotchCoeffs(centerFreq, Q, coeffs);
    
    // Initialize ARM DSP biquad filter structure
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
    
    return filterStates;
}

float SignalFilters::processSample(float input, FilterStates &filterStates) {
    float output;
    
    // Process a single sample through the filter using ARM DSP functions
    arm_biquad_cascade_df1_f32(&filterStates.instance, &input, &output, 1);
    
    return output;
}

void SignalFilters::updateLowPassFilter(FilterStates &filterStates, float cutoffFreq) {
    float coeffs[5];
    calculateLowPassCoeffs(cutoffFreq, coeffs);
    
    // Reset state
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Reinitialize the filter with new coefficients
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
}

void SignalFilters::updateHighPassFilter(FilterStates &filterStates, float cutoffFreq) {
    float coeffs[5];
    calculateHighPassCoeffs(cutoffFreq, coeffs);
    
    // Reset state
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Reinitialize the filter with new coefficients
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
}

void SignalFilters::updateBandPassFilter(FilterStates &filterStates, float lowFreq, float highFreq) {
    float coeffs[5];
    calculateBandPassCoeffs(lowFreq, highFreq, coeffs);
    
    // Reset state
    for (int i = 0; i < 4; i++) {
        filterStates.state[i] = 0.0f;
    }
    
    // Reinitialize the filter with new coefficients
    arm_biquad_cascade_df1_init_f32(&filterStates.instance, 1, coeffs, filterStates.state);
}

// Implementation of coefficient calculation methods

void SignalFilters::calculateLowPassCoeffs(float cutoffFreq, float *coeffs) {
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * cutoffFreq / _sampleRate;
    float alpha = sin(omega) / (2.0f * 0.7071f); // Q factor = 0.7071 for Butterworth
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // [b0, b1, b2, a1, a2] format required by ARM DSP biquad filter
    coeffs[0] = (1.0f - cosw) / (2.0f * a0);       // b0
    coeffs[1] = (1.0f - cosw) / a0;                // b1
    coeffs[2] = (1.0f - cosw) / (2.0f * a0);       // b2
    coeffs[3] = -(-2.0f * cosw / a0);              // -a1 (ARM DSP uses -a1, -a2)
    coeffs[4] = -((1.0f - alpha) / a0);            // -a2
}

void SignalFilters::calculateHighPassCoeffs(float cutoffFreq, float *coeffs) {
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * cutoffFreq / _sampleRate;
    float alpha = sin(omega) / (2.0f * 0.7071f); // Q factor = 0.7071 for Butterworth
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // [b0, b1, b2, a1, a2] format required by ARM DSP biquad filter
    coeffs[0] = (1.0f + cosw) / (2.0f * a0);       // b0
    coeffs[1] = -(1.0f + cosw) / a0;               // b1
    coeffs[2] = (1.0f + cosw) / (2.0f * a0);       // b2
    coeffs[3] = -(-2.0f * cosw / a0);              // -a1
    coeffs[4] = -((1.0f - alpha) / a0);            // -a2
}

void SignalFilters::calculateBandPassCoeffs(float lowFreq, float highFreq, float *coeffs) {
    // Calculate center frequency and bandwidth
    float centerFreq = sqrt(lowFreq * highFreq);
    float bandwidth = highFreq - lowFreq;
    
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float alpha = sin(omega) * sinh(log(2.0f) / 2.0f * bandwidth / centerFreq * omega / sin(omega));
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // [b0, b1, b2, a1, a2] format required by ARM DSP biquad filter
    coeffs[0] = alpha / a0;                        // b0
    coeffs[1] = 0;                                 // b1
    coeffs[2] = -alpha / a0;                       // b2
    coeffs[3] = -(-2.0f * cosw / a0);              // -a1
    coeffs[4] = -((1.0f - alpha) / a0);            // -a2
}

void SignalFilters::calculateBandStopCoeffs(float lowFreq, float highFreq, float *coeffs) {
    // Calculate center frequency and bandwidth
    float centerFreq = sqrt(lowFreq * highFreq);
    float bandwidth = highFreq - lowFreq;
    
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float alpha = sin(omega) * sinh(log(2.0f) / 2.0f * bandwidth / centerFreq * omega / sin(omega));
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // [b0, b1, b2, a1, a2] format required by ARM DSP biquad filter
    coeffs[0] = 1.0f / a0;                         // b0
    coeffs[1] = -2.0f * cosw / a0;                 // b1
    coeffs[2] = 1.0f / a0;                         // b2
    coeffs[3] = -(-2.0f * cosw / a0);              // -a1
    coeffs[4] = -((1.0f - alpha) / a0);            // -a2
}

void SignalFilters::calculateNotchCoeffs(float centerFreq, float Q, float *coeffs) {
    // Normalized frequency (0 to π)
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float alpha = sin(omega) / (2.0f * Q);
    
    float cosw = cos(omega);
    float a0 = 1.0f + alpha;
    
    // [b0, b1, b2, a1, a2] format required by ARM DSP biquad filter
    coeffs[0] = 1.0f / a0;                         // b0
    coeffs[1] = -2.0f * cosw / a0;                 // b1
    coeffs[2] = 1.0f / a0;                         // b2
    coeffs[3] = -(-2.0f * cosw / a0);              // -a1
    coeffs[4] = -((1.0f - alpha) / a0);            // -a2
}