// SignalFilters.cpp
#include "SignalFilters.h"

SignalFilters::SignalFilters(float sampleRate) : _sampleRate(sampleRate) {
}

SignalFilters::BiquadCoefficients SignalFilters::createLowPassFilter(float cutoffFreq) {
    BiquadCoefficients coeffs;
    
    // Ensure cutoff frequency is within valid range
    if (cutoffFreq <= 0.0f) cutoffFreq = 0.1f;
    if (cutoffFreq >= _sampleRate/2.0f) cutoffFreq = _sampleRate/2.0f * 0.99f;
    
    // Normalize frequency
    float omega = 2.0f * PI * cutoffFreq / _sampleRate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0f * 0.7071f); // Q factor = 0.7071 for Butterworth
    
    // Prevent division by zero
    if (alpha < 0.0001f) alpha = 0.0001f;
    
    // Calculate coefficients (bilinear transform)
    float a0 = 1.0f + alpha;
    
    // Prevent division by zero
    if (fabs(a0) < 0.0001f) a0 = 0.0001f;
    
    // Normalized coefficients
    coeffs.b0 = (1.0f - cs) / (2.0f * a0);
    coeffs.b1 = (1.0f - cs) / a0;
    coeffs.b2 = (1.0f - cs) / (2.0f * a0);
    coeffs.a1 = (-2.0f * cs) / a0;
    coeffs.a2 = (1.0f - alpha) / a0;
    
    // Ensure coefficients aren't NaN
    if (isnan(coeffs.b0)) coeffs.b0 = 0.0f;
    if (isnan(coeffs.b1)) coeffs.b1 = 0.0f;
    if (isnan(coeffs.b2)) coeffs.b2 = 0.0f;
    if (isnan(coeffs.a1)) coeffs.a1 = 0.0f;
    if (isnan(coeffs.a2)) coeffs.a2 = 0.0f;
    
    return coeffs;
}

SignalFilters::BiquadCoefficients SignalFilters::createHighPassFilter(float cutoffFreq) {
    BiquadCoefficients coeffs;
    
    // Ensure cutoff frequency is within valid range
    if (cutoffFreq <= 0.0f) cutoffFreq = 0.1f;
    if (cutoffFreq >= _sampleRate/2.0f) cutoffFreq = _sampleRate/2.0f * 0.99f;
    
    // Normalize frequency
    float omega = 2.0f * PI * cutoffFreq / _sampleRate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0f * 0.7071f); // Q factor = 0.7071 for Butterworth
    
    // Prevent division by zero
    if (alpha < 0.0001f) alpha = 0.0001f;
    
    // Calculate coefficients (bilinear transform)
    float a0 = 1.0f + alpha;
    
    // Prevent division by zero
    if (fabs(a0) < 0.0001f) a0 = 0.0001f;
    
    // Normalized coefficients
    coeffs.b0 = (1.0f + cs) / (2.0f * a0);
    coeffs.b1 = -(1.0f + cs) / a0;
    coeffs.b2 = (1.0f + cs) / (2.0f * a0);
    coeffs.a1 = (-2.0f * cs) / a0;
    coeffs.a2 = (1.0f - alpha) / a0;
    
    // Ensure coefficients aren't NaN
    if (isnan(coeffs.b0)) coeffs.b0 = 0.0f;
    if (isnan(coeffs.b1)) coeffs.b1 = 0.0f;
    if (isnan(coeffs.b2)) coeffs.b2 = 0.0f;
    if (isnan(coeffs.a1)) coeffs.a1 = 0.0f;
    if (isnan(coeffs.a2)) coeffs.a2 = 0.0f;
    
    return coeffs;
}

SignalFilters::BiquadCoefficients SignalFilters::createBandPassFilter(float lowFreq, float highFreq) {
    BiquadCoefficients coeffs;
    
    // Ensure frequencies are valid
    if (lowFreq <= 0.0f) lowFreq = 0.1f;
    if (highFreq >= _sampleRate/2.0f) highFreq = _sampleRate/2.0f * 0.99f;
    if (lowFreq >= highFreq) {
        float temp = lowFreq;
        lowFreq = highFreq * 0.5f;
        highFreq = temp * 2.0f;
    }
    
    // Calculate center frequency and bandwidth
    float centerFreq = sqrt(lowFreq * highFreq);
    float bandwidth = highFreq - lowFreq;
    
    // Normalize frequency
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float sn = sin(omega);
    float cs = cos(omega);
    
    // Calculate alpha (for bandwidth)
    float alpha = sn * sinh(log(2.0f) / 2.0f * bandwidth / centerFreq * omega / sn);
    
    // Prevent division by zero or NaN
    if (isnan(alpha) || alpha < 0.0001f) alpha = 0.0001f;
    
    // Calculate coefficients
    float a0 = 1.0f + alpha;
    
    // Prevent division by zero
    if (fabs(a0) < 0.0001f) a0 = 0.0001f;
    
    // Normalized coefficients
    coeffs.b0 = alpha / a0;
    coeffs.b1 = 0.0f;
    coeffs.b2 = -alpha / a0;
    coeffs.a1 = -2.0f * cs / a0;
    coeffs.a2 = (1.0f - alpha) / a0;
    
    // Ensure coefficients aren't NaN
    if (isnan(coeffs.b0)) coeffs.b0 = 0.0f;
    if (isnan(coeffs.b1)) coeffs.b1 = 0.0f;
    if (isnan(coeffs.b2)) coeffs.b2 = 0.0f;
    if (isnan(coeffs.a1)) coeffs.a1 = 0.0f;
    if (isnan(coeffs.a2)) coeffs.a2 = 0.0f;
    
    return coeffs;
}

SignalFilters::BiquadCoefficients SignalFilters::createBandStopFilter(float lowFreq, float highFreq) {
    BiquadCoefficients coeffs;
    
    // Ensure frequencies are valid
    if (lowFreq <= 0.0f) lowFreq = 0.1f;
    if (highFreq >= _sampleRate/2.0f) highFreq = _sampleRate/2.0f * 0.99f;
    if (lowFreq >= highFreq) {
        float temp = lowFreq;
        lowFreq = highFreq * 0.5f;
        highFreq = temp * 2.0f;
    }
    
    // Calculate center frequency and bandwidth
    float centerFreq = sqrt(lowFreq * highFreq);
    float bandwidth = highFreq - lowFreq;
    
    // Normalize frequency
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float sn = sin(omega);
    float cs = cos(omega);
    
    // Calculate alpha (for bandwidth)
    float alpha = sn * sinh(log(2.0f) / 2.0f * bandwidth / centerFreq * omega / sn);
    
    // Prevent division by zero or NaN
    if (isnan(alpha) || alpha < 0.0001f) alpha = 0.0001f;
    
    // Calculate coefficients
    float a0 = 1.0f + alpha;
    
    // Prevent division by zero
    if (fabs(a0) < 0.0001f) a0 = 0.0001f;
    
    // Normalized coefficients
    coeffs.b0 = 1.0f / a0;
    coeffs.b1 = -2.0f * cs / a0;
    coeffs.b2 = 1.0f / a0;
    coeffs.a1 = -2.0f * cs / a0;
    coeffs.a2 = (1.0f - alpha) / a0;
    
    // Ensure coefficients aren't NaN
    if (isnan(coeffs.b0)) coeffs.b0 = 0.0f;
    if (isnan(coeffs.b1)) coeffs.b1 = 0.0f;
    if (isnan(coeffs.b2)) coeffs.b2 = 0.0f;
    if (isnan(coeffs.a1)) coeffs.a1 = 0.0f;
    if (isnan(coeffs.a2)) coeffs.a2 = 0.0f;
    
    return coeffs;
}

SignalFilters::BiquadCoefficients SignalFilters::createNotchFilter(float centerFreq, float Q) {
    BiquadCoefficients coeffs;
    
    // Ensure parameters are valid
    if (centerFreq <= 0.0f || centerFreq >= _sampleRate/2.0f) {
        centerFreq = _sampleRate/4.0f; // Default to 1/4 of sample rate
    }
    
    if (Q <= 0.1f) Q = 0.1f;
    if (Q > 100.0f) Q = 100.0f;
    
    // Normalize frequency
    float omega = 2.0f * PI * centerFreq / _sampleRate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0f * Q);
    
    // Prevent division by zero
    if (alpha < 0.0001f) alpha = 0.0001f;
    
    // Calculate coefficients
    float a0 = 1.0f + alpha;
    
    // Prevent division by zero
    if (fabs(a0) < 0.0001f) a0 = 0.0001f;
    
    // Normalized coefficients
    coeffs.b0 = 1.0f / a0;
    coeffs.b1 = -2.0f * cs / a0;
    coeffs.b2 = 1.0f / a0;
    coeffs.a1 = -2.0f * cs / a0;
    coeffs.a2 = (1.0f - alpha) / a0;
    
    // Ensure coefficients aren't NaN
    if (isnan(coeffs.b0)) coeffs.b0 = 0.0f;
    if (isnan(coeffs.b1)) coeffs.b1 = 0.0f;
    if (isnan(coeffs.b2)) coeffs.b2 = 0.0f;
    if (isnan(coeffs.a1)) coeffs.a1 = 0.0f;
    if (isnan(coeffs.a2)) coeffs.a2 = 0.0f;
    
    return coeffs;
}

float SignalFilters::applyFilter(float input, const BiquadCoefficients &coeffs, FilterState &state) {
    // Direct Form I implementation (more stable for low frequencies)
    float output = coeffs.b0 * input + coeffs.b1 * state.x1 + coeffs.b2 * state.x2 
                 - coeffs.a1 * state.y1 - coeffs.a2 * state.y2;
    
    // Update state
    state.x2 = state.x1;
    state.x1 = input;
    state.y2 = state.y1;
    state.y1 = output;
    
    // Detect and prevent NaN propagation
    if (isnan(output)) {
        output = 0.0f;
        // Reset filter state when NaN occurs
        state.x1 = 0.0f;
        state.x2 = 0.0f;
        state.y1 = 0.0f;
        state.y2 = 0.0f;
    }
    
    // Apply limiter to prevent unstable growth
    const float MAX_VALUE = 1000000.0f;
    if (output > MAX_VALUE) output = MAX_VALUE;
    if (output < -MAX_VALUE) output = -MAX_VALUE;
    
    return output;
}

float SignalFilters::applyDCRemoval(float input, float &prevInput, float &prevOutput, float alpha) {
    // High-pass filter for DC removal
    float output = alpha * prevOutput + alpha * (input - prevInput);
    
    // Update state
    prevInput = input;
    prevOutput = output;
    
    // Prevent NaN
    if (isnan(output)) {
        output = 0.0f;
        prevInput = 0.0f;
        prevOutput = 0.0f;
    }
    
    // Apply limiter to prevent unstable growth
    const float MAX_VALUE = 1000000.0f;
    if (output > MAX_VALUE) output = MAX_VALUE;
    if (output < -MAX_VALUE) output = -MAX_VALUE;
    
    return output;
}