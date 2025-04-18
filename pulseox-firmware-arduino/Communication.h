// Communication.h
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "MAX30102_Handler.h"
#include "SignalFilters.h"
#include "core.h"

class Communication {
public:
  Communication();

  // Initialize serial communication
  void begin(unsigned long baudRate);

  // Handle serial commands
  void handleSerialCommands(
    bool &samplingActive,
    PulseOxConfig &config,
    MAX30102_Handler &sensor,
    SignalFilters &filters,
    float &breathingLowCutoff,
    float &breathingHighCutoff,
    float &heartRateLowCutoff,
    float &heartRateHighCutoff
  );

  // Output data in CSV format
  void outputData(uint32_t irValue, float irFiltered_BR, float irFiltered_HR);

  // Print sensor configuration
  void printSensorConfig(PulseOxConfig &config);

  // Print filter settings
  void printFilterSettings(float breathingLowCutoff, float breathingHighCutoff,
                         float heartRateLowCutoff, float heartRateHighCutoff);

  // Print menu
  void printMenu();

  // Print CSV header
  void printCSVHeader();

  // Reset notification
  void notifyReset();

  // Sampling status notification
  void notifySamplingStatus(bool active);

  // Adjustment functions for filter parameters
  float adjustBreathingLowCutoff(bool &samplingActive, float currentLow, float currentHigh);
  float adjustBreathingHighCutoff(bool &samplingActive, float currentLow, float currentHigh, float heartRateLow);
  float adjustHeartRateLowCutoff(bool &samplingActive, float breathingHigh, float currentLow, float currentHigh);
  float adjustHeartRateHighCutoff(bool &samplingActive, float currentLow, float currentHigh);

private:
  // Output buffer
  char _outputBuffer[128];
};

#endif // COMMUNICATION_H
