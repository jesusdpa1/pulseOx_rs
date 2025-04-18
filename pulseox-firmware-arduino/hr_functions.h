// hr_functions.h
#ifndef HR_FUNCTIONS_H
#define HR_FUNCTIONS_H

#include <Arduino.h>

enum PeakDetectionAlgorithm {
  ADAPTIVE_THRESHOLD,
  DERIVATIVE_ZERO_CROSSING,
  PARABOLIC_INTERPOLATION,
  PAN_TOMPKINS
};

class HeartRateDetector {
public:
  HeartRateDetector();

  // Initialize with a specific algorithm
  void begin(PeakDetectionAlgorithm algorithm = ADAPTIVE_THRESHOLD, float sampleRate = 100.0f);

  // Process a new sample and detect peaks
  bool processSample(float newSample, uint32_t timestamp);

  // Get the current heart rate in BPM
  float getHeartRate();

  // Get confidence level of the current heart rate estimate (0.0-1.0)
  float getConfidence();

  // Reset the detector state
  void reset();

  // Change algorithm dynamically
  void setAlgorithm(PeakDetectionAlgorithm newAlgorithm);

  // Set the sample rate
  void setSampleRate(float newSampleRate);

private:
  // Algorithm selection
  PeakDetectionAlgorithm _currentAlgorithm;
  float _sampleRate;

  // Common variables
  float _lastPeak;
  uint32_t _lastPeakTime;
  uint32_t _previousPeakTimes[8]; // Store last 8 peak times for averaging
  uint8_t _peakTimeIndex;
  uint8_t _peakCount;
  float _currentHeartRate;
  float _confidence;

  // Adaptive threshold variables
  float _threshold;
  float _prevSamples[3];

  // Derivative method variables
  float _prevSample;
  float _prevDerivative;

  // Parabolic interpolation variables
  float _samples[3];

  // Pan-Tompkins variables
  float _ptBuffer[4];
  int _ptIndex;
  float _ptIntegral;
  float _prevIntegral;

  // Algorithm implementations
  bool detectPeakAdaptiveThreshold(float newSample, uint32_t timestamp);
  bool detectPeakDerivative(float newSample, uint32_t timestamp);
  bool detectPeakParabolic(float newSample, uint32_t timestamp);
  bool detectPeakPanTompkins(float newSample, uint32_t timestamp);

  // Heart rate calculation
  void calculateHeartRate(uint32_t currentPeakTime);

  // Helper functions
  bool isValidPeakInterval(uint32_t currentTime);
  void updateConfidence(uint32_t currentTime);
};

#endif // HR_FUNCTIONS_H
