// hr_functions.cpp
#include "hr_functions.h"

HeartRateDetector::HeartRateDetector() {
  _currentAlgorithm = ADAPTIVE_THRESHOLD;
  _sampleRate = 100.0f;
  reset();
}

void HeartRateDetector::begin(PeakDetectionAlgorithm algorithm, float sampleRate) {
  _currentAlgorithm = algorithm;
  _sampleRate = sampleRate;
  reset();
}

void HeartRateDetector::reset() {
  // Reset common variables
  _lastPeak = 0.0f;
  _lastPeakTime = 0;
  _currentHeartRate = 0.0f;
  _confidence = 0.0f;
  _peakTimeIndex = 0;
  _peakCount = 0;

  for (int i = 0; i < 8; i++) {
    _previousPeakTimes[i] = 0;
  }

  // Reset algorithm-specific variables
  // Adaptive threshold
  _threshold = 0.0f;
  for (int i = 0; i < 3; i++) {
    _prevSamples[i] = 0.0f;
  }

  // Derivative
  _prevSample = 0.0f;
  _prevDerivative = 0.0f;

  // Parabolic
  for (int i = 0; i < 3; i++) {
    _samples[i] = 0.0f;
  }

  // Pan-Tompkins
  for (int i = 0; i < 4; i++) {
    _ptBuffer[i] = 0.0f;
  }
  _ptIndex = 0;
  _ptIntegral = 0.0f;
  _prevIntegral = 0.0f;
}

bool HeartRateDetector::processSample(float newSample, uint32_t timestamp) {
  bool peakDetected = false;

  // Call the appropriate algorithm based on the selection
  switch (_currentAlgorithm) {
    case ADAPTIVE_THRESHOLD:
      peakDetected = detectPeakAdaptiveThreshold(newSample, timestamp);
      break;
    case DERIVATIVE_ZERO_CROSSING:
      peakDetected = detectPeakDerivative(newSample, timestamp);
      break;
    case PARABOLIC_INTERPOLATION:
      peakDetected = detectPeakParabolic(newSample, timestamp);
      break;
    case PAN_TOMPKINS:
      peakDetected = detectPeakPanTompkins(newSample, timestamp);
      break;
    default:
      peakDetected = detectPeakAdaptiveThreshold(newSample, timestamp);
      break;
  }

  // Calculate heart rate if a peak was detected
  if (peakDetected) {
    calculateHeartRate(timestamp);
  }

  // Update confidence based on regularity and timing
  if (timestamp - _lastPeakTime > 2000) { // If no peak for 2 seconds
    _confidence *= 0.95f; // Decay confidence
  }

  return peakDetected;
}

float HeartRateDetector::getHeartRate() {
  return _currentHeartRate;
}

float HeartRateDetector::getConfidence() {
  return _confidence;
}

void HeartRateDetector::setAlgorithm(PeakDetectionAlgorithm newAlgorithm) {
  if (newAlgorithm != _currentAlgorithm) {
    _currentAlgorithm = newAlgorithm;
    reset(); // Reset state when changing algorithms
  }
}

void HeartRateDetector::setSampleRate(float newSampleRate) {
  if (newSampleRate > 0) {
    _sampleRate = newSampleRate;
  }
}

// Algorithm 1: Adaptive Thresholding
bool HeartRateDetector::detectPeakAdaptiveThreshold(float newSample, uint32_t timestamp) {
  // Gradually adjust threshold (slow decay, fast attack)
  if (_threshold < newSample) {
    _threshold = _threshold * 0.7f + newSample * 0.3f; // Fast attack
  } else {
    _threshold = _threshold * 0.95f + newSample * 0.05f; // Slow decay
  }

  // Ensure threshold is not zero to prevent division issues
  if (_threshold < 0.01f) _threshold = 0.01f;

  // Peak detected when sample exceeds threshold by factor and is larger than last few samples
  bool isPeak = (newSample > (_threshold * 0.7f)) &&
                (newSample > _prevSamples[0]) &&
                (newSample > _prevSamples[1]) &&
                (newSample > _prevSamples[2]);

  // Update history
  _prevSamples[2] = _prevSamples[1];
  _prevSamples[1] = _prevSamples[0];
  _prevSamples[0] = newSample;

  // Check if enough time has passed since last peak (prevents double-counting)
  if (isPeak && isValidPeakInterval(timestamp)) {
    _lastPeak = newSample;
    _lastPeakTime = timestamp;
    return true;
  }

  return false;
}

// Algorithm 2: Zero-Crossing Derivative Method
bool HeartRateDetector::detectPeakDerivative(float newSample, uint32_t timestamp) {
  // Calculate current derivative
  float currentDerivative = newSample - _prevSample;

  // Zero crossing from positive to negative indicates a peak
  bool isPeak = (_prevDerivative > 0.1f) && (currentDerivative <= 0);

  // Store for next iteration
  _prevSample = newSample;
  _prevDerivative = currentDerivative;

  // Additional validation to prevent noise detection
  if (isPeak && isValidPeakInterval(timestamp) && (newSample > 0.3f * _threshold)) {
    _lastPeak = newSample;
    _lastPeakTime = timestamp;
    // Update threshold for validation
    _threshold = _threshold * 0.7f + newSample * 0.3f;
    return true;
  }

  return false;
}

// Algorithm 3: Three-Point Parabolic Interpolation
bool HeartRateDetector::detectPeakParabolic(float newSample, uint32_t timestamp) {
  // Update sample buffer
  _samples[0] = _samples[1];
  _samples[1] = _samples[2];
  _samples[2] = newSample;

  // Update threshold for validation
  if (newSample > _threshold) {
    _threshold = _threshold * 0.7f + newSample * 0.3f;
  } else {
    _threshold = _threshold * 0.95f + newSample * 0.05f;
  }

  // Check if middle point is higher than neighbors
  bool isPeak = (_samples[1] > _samples[0]) &&
                (_samples[1] > _samples[2]) &&
                (_samples[1] > 0.5f * _threshold);

  if (isPeak && isValidPeakInterval(timestamp)) {
    // Parabolic interpolation for sub-sample peak detection
    float denominator = _samples[0] - 2.0f * _samples[1] + _samples[2];

    // Avoid division by zero
    if (abs(denominator) > 0.01f) {
      float alpha = 0.5f * ((_samples[0] - _samples[2]) / denominator);

      // Constrain alpha to valid range [-0.5, 0.5]
      if (alpha > 0.5f) alpha = 0.5f;
      if (alpha < -0.5f) alpha = -0.5f;

      // Calculate precise peak value
      float peakValue = _samples[1] - 0.25f * (_samples[0] - _samples[2]) * alpha;
      _lastPeak = peakValue;
    } else {
      _lastPeak = _samples[1];
    }

    _lastPeakTime = timestamp;
    return true;
  }

  return false;
}

// Algorithm 4: Pan-Tompkins Algorithm (Simplified)
bool HeartRateDetector::detectPeakPanTompkins(float newSample, uint32_t timestamp) {
  // Calculate derivative
  float derivative = newSample - _prevSample;
  _prevSample = newSample;

  // Square the derivative to emphasize larger slopes
  float squared = derivative * derivative;

  // Moving window integration
  _ptIntegral = _ptIntegral - _ptBuffer[_ptIndex] + squared;
  _ptBuffer[_ptIndex] = squared;
  _ptIndex = (_ptIndex + 1) % 4;

  // Adaptive thresholding on integrated signal
  float integralThreshold = _prevIntegral * 1.1f;  // 10% above previous

  // Detect rising edge crossing threshold
  bool isPeak = (_ptIntegral > integralThreshold) && (_ptIntegral > 0.5f);

  _prevIntegral = _ptIntegral * 0.9f + _prevIntegral * 0.1f;  // Smooth for next iteration

  // Validation with timing constraint
  if (isPeak && isValidPeakInterval(timestamp)) {
    _lastPeak = _ptIntegral;
    _lastPeakTime = timestamp;
    return true;
  }

  return false;
}

// Helper function to validate peak intervals based on expected HR range
bool HeartRateDetector::isValidPeakInterval(uint32_t currentTime) {
  // For rodent heart rates (250-750 BPM)
  // Minimum interval would be 60,000ms / 750 = 80ms
  // Maximum interval would be 60,000ms / 250 = 240ms
  uint32_t minInterval = 60;  // Allow slightly faster for safety (1000 BPM max)
  uint32_t maxInterval = 300; // Allow slightly slower for safety (200 BPM min)

  uint32_t timeSinceLastPeak = currentTime - _lastPeakTime;

  return (timeSinceLastPeak > minInterval) &&
         (_lastPeakTime == 0 || timeSinceLastPeak < maxInterval || _peakCount < 4);
}

// Calculate heart rate from peak times
void HeartRateDetector::calculateHeartRate(uint32_t currentPeakTime) {
  if (_lastPeakTime == 0) {
    // First peak, can't calculate rate yet
    _previousPeakTimes[0] = currentPeakTime;
    _peakTimeIndex = 1;
    _peakCount = 1;
    return;
  }

  // Store this peak time in circular buffer
  _previousPeakTimes[_peakTimeIndex] = currentPeakTime;
  _peakTimeIndex = (_peakTimeIndex + 1) % 8;
  if (_peakCount < 8) _peakCount++;

  // Calculate average interval if we have at least 3 peaks
  if (_peakCount >= 3) {
    uint32_t totalInterval = 0;
    uint32_t intervalsUsed = 0;

    // Calculate the sum of valid intervals
    for (int i = 0; i < _peakCount - 1; i++) {
      int currentIdx = (_peakTimeIndex - 1 - i + 8) % 8;
      int previousIdx = (_peakTimeIndex - 2 - i + 8) % 8;

      uint32_t interval = _previousPeakTimes[currentIdx] - _previousPeakTimes[previousIdx];

      // Only use intervals in valid range
      if (interval >= 60 && interval <= 300) {
        totalInterval += interval;
        intervalsUsed++;
      }
    }

    // Calculate heart rate if we have valid intervals
    if (intervalsUsed > 0) {
      float avgInterval = (float)totalInterval / intervalsUsed;
      float newHeartRate = 60000.0f / avgInterval;

      // Smooth the heart rate for stability
      if (_currentHeartRate == 0) {
        _currentHeartRate = newHeartRate;
      } else {
        _currentHeartRate = _currentHeartRate * 0.7f + newHeartRate * 0.3f;
      }

      // Update confidence based on regularity
      updateConfidence(currentPeakTime);
    }
  }
}

// Update confidence level based on regularity of peaks
void HeartRateDetector::updateConfidence(uint32_t currentTime) {
  if (_peakCount < 3) {
    _confidence = 0.3f; // Low confidence with few peaks
    return;
  }

  // Calculate variance of intervals to assess regularity
  float mean = 0;
  float variance = 0;
  int validIntervals = 0;

  // Calculate mean interval
  for (int i = 0; i < _peakCount - 1; i++) {
    int currentIdx = (_peakTimeIndex - 1 - i + 8) % 8;
    int previousIdx = (_peakTimeIndex - 2 - i + 8) % 8;

    uint32_t interval = _previousPeakTimes[currentIdx] - _previousPeakTimes[previousIdx];
    if (interval >= 60 && interval <= 300) {
      mean += interval;
      validIntervals++;
    }
  }

  if (validIntervals > 0) {
    mean /= validIntervals;

    // Calculate variance
    for (int i = 0; i < _peakCount - 1; i++) {
      int currentIdx = (_peakTimeIndex - 1 - i + 8) % 8;
      int previousIdx = (_peakTimeIndex - 2 - i + 8) % 8;

      uint32_t interval = _previousPeakTimes[currentIdx] - _previousPeakTimes[previousIdx];
      if (interval >= 60 && interval <= 300) {
        float diff = interval - mean;
        variance += diff * diff;
      }
    }

    if (validIntervals > 1) {
      variance /= (validIntervals - 1);
    }

    // CV (Coefficient of Variation) = standard deviation / mean
    float cv = sqrt(variance) / mean;

    // Map CV to confidence (lower CV = higher confidence)
    // CV < 0.1 is very regular, CV > 0.3 is irregular
    float newConfidence = 1.0f - constrain(cv / 0.3f, 0.0f, 0.9f);

    // Smooth confidence update
    _confidence = _confidence * 0.8f + newConfidence * 0.2f;

    // Cap confidence based on number of peaks observed
    if (_peakCount < 5) {
      _confidence = min(_confidence, 0.7f);
    }
  }
}
