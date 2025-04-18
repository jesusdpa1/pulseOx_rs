// pulseox-firmware-arduino.ino with communication module

#include "MAX30102_Handler.h"
#include "SignalFilters.h"
#include "core.h"
#include "Communication.h"
// In your main Arduino file
// Define pins
#define MAX30102_INT_PIN 2  // Digital pin 2 for interrupt (not connected for now)

// Create instances
MAX30102_Handler sensor;
PulseOxConfig config;
SignalFilters filters(100.0f); // Placeholder sample rate, will update in setup
Communication comm;

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 10; // Variable for sample interval

// Filter coefficient structures
SignalFilters::BiquadCoefficients breathingFilter;    // Band-pass filter for breathing component
SignalFilters::BiquadCoefficients heartRateFilter;    // Band-pass filter for heart rate component

// Filter states
SignalFilters::FilterState breathingFilterState;
SignalFilters::FilterState heartRateFilterState;

// DC removal filter states
float dcRemovalInput = 0.0f;
float dcRemovalOutput = 0.0f;

// Filter parameter variables
float breathingLowCutoff = 0.1f;      // Default low cutoff for breathing (Hz)
float breathingHighCutoff = 0.3f;     // Default high cutoff for breathing (Hz)
float heartRateLowCutoff = 0.5f;      // Default low cutoff for heart rate (Hz)
float heartRateHighCutoff = 1.0f;     // Default high cutoff for heart rate (Hz)

// Sample rate value (to be set in setup)
float actualSampleRate = 100.0f;

// Flag to start/stop sampling
bool samplingActive = false;

void setup() {
  // Initialize serial communication
  comm.begin(115200);

  // Initialize I2C
  Wire.begin();

  // Configure sensor with specific settings
  config.sensorMode = MODE_SPO2;       // Use SpO2 mode (RED + IR LEDs)
  config.sampleRate = SAMPLERATE_400;  // 800 samples per second
  config.pulseWidth = PULSEWIDTH_69;   // 69μs pulse width
  config.adcRange = ADCRANGE_16384;    // Full range
  config.sampleAvg = SAMPLEAVG_2;      // Average 4 samples
  config.redLedAmplitude = 60;         // LED power (0-255)
  config.irLedAmplitude = 60;          // LED power (0-255)
  config.redLedEnabled = true;
  config.irLedEnabled = true;
  config.fifoAlmostFull = 4;           // Generate interrupt when FIFO has 4 samples

  // Initialize sensor
  if (!sensor.begin()) {
    Serial.println("ERROR: MAX30102 not found or failed to initialize!");
    Serial.println("1. Check if sensor is connected properly");
    Serial.println("2. Check power supply (3.3V required)");
    Serial.println("3. Check I2C address (default: 0x57)");
    while (1); // Halt execution
  }

  Serial.println("MAX30102 found!");

  // Apply configuration
  sensor.setup(config);

  // Print configuration
  comm.printSensorConfig(config);

  // Set up filters based on sample rate
  switch (config.sampleRate) {
    case SAMPLERATE_50: actualSampleRate = 50.0f; break;
    case SAMPLERATE_100: actualSampleRate = 100.0f; break;
    case SAMPLERATE_200: actualSampleRate = 200.0f; break;
    case SAMPLERATE_400: actualSampleRate = 400.0f; break;
    case SAMPLERATE_800: actualSampleRate = 800.0f; break;
    case SAMPLERATE_1000: actualSampleRate = 1000.0f; break;
    case SAMPLERATE_1600: actualSampleRate = 1600.0f; break;
    case SAMPLERATE_3200: actualSampleRate = 3200.0f; break;
    default: actualSampleRate = 400.0f; break;
  }

  // Update filter instance with actual sample rate
  filters = SignalFilters(actualSampleRate);

  // Design initial filters
  designFilters();

  // Reset filter states
  resetFilterStates();

  // Calculate sample interval in ms
  sampleInterval = 1000 / (unsigned long)actualSampleRate;

  // Print menu
  comm.printMenu();

  // Print CSV header
  comm.printCSVHeader();

  // Clear any unread data
  sensor.clearFIFO();

  // Start with sampling disabled
  samplingActive = false;
}

void loop() {
  // Check for serial commands
  comm.handleSerialCommands(
    samplingActive, config, sensor, filters,
    breathingLowCutoff, breathingHighCutoff,
    heartRateLowCutoff, heartRateHighCutoff
  );

  // Read and process sensor data if active
  if (samplingActive && (millis() - lastSampleTime >= sampleInterval)) {
    lastSampleTime = millis();

    // Read data from sensor
    uint32_t redValue, irValue;

    if (sensor.available() > 0) {
      if (sensor.getRawData(&redValue, &irValue)) {
        // Apply filters to IR signal (typically cleaner than RED for PPG)
        float irFloat = (float)irValue;
        float redFloat = (float)((int)redValue);
        // First remove DC component
        float irDC = filters.applyDCRemoval(irFloat, dcRemovalInput, dcRemovalOutput);
        // float redDC = filters.applyDCRemoval(redFloat, dcRemovalInput, dcRemovalOutput);
        // Then apply the bandpass filters
        
        float irFiltered_HR = filters.applyFilter(irDC, heartRateFilter, heartRateFilterState);
        float irFiltered_BR = filters.applyFilter(redFloat, breathingFilter, breathingFilterState);
        // Output data
        comm.outputData(irValue, irFiltered_BR, irFiltered_HR);
      }
    }
  }
}

// Design filters with current parameters
void designFilters() {
  // Design breathing band-pass filter (0.1-0.5 Hz typical for respiration)
  breathingFilter = filters.createBandPassFilter(breathingLowCutoff, breathingHighCutoff);

  // Design heart rate band-pass filter (0.5-5 Hz typical for heart rate)
  heartRateFilter = filters.createBandPassFilter(heartRateLowCutoff, heartRateHighCutoff);

  // Reset filter states
  resetFilterStates();

  // Print current filter settings
  comm.printFilterSettings(
    breathingLowCutoff, breathingHighCutoff,
    heartRateLowCutoff, heartRateHighCutoff
  );
}

// Reset filter states when parameters change
void resetFilterStates() {
  breathingFilterState = SignalFilters::FilterState();
  heartRateFilterState = SignalFilters::FilterState();
  dcRemovalInput = 0.0f;
  dcRemovalOutput = 0.0f;
}
