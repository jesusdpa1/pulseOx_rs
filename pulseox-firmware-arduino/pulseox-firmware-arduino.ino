// MAX30102_RawData_Test.ino with adjustable band-pass filters

#include "MAX30102_Handler.h"
#include "SignalFilters.h"
#include "core.h"

// Define pins
#define MAX30102_INT_PIN 2  // Digital pin 2 for interrupt (not connected for now)

// Create instances
MAX30102_Handler sensor;
PulseOxConfig config;
SignalFilters filters(100.0f); // Placeholder sample rate, will update in setup

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 10; // Variable for sample interval

// Filter coefficients
FilterCoefficients breathingFilter;    // Band-pass filter for breathing component
FilterCoefficients heartRateFilter;    // Band-pass filter for heart rate component

// Filter states
float breathingFilterState[4] = {0};
float heartRateFilterState[4] = {0};

// Filter parameter variables
float breathingLowCutoff = 0.11f;      // Default low cutoff for breathing (Hz)
float breathingHighCutoff = 0.5f;     // Default high cutoff for breathing (Hz)
float heartRateLowCutoff = 0.2f;      // Default low cutoff for heart rate (Hz)
float heartRateHighCutoff = 5.0f;     // Default high cutoff for heart rate (Hz)

// Sample rate value (to be set in setup)
float actualSampleRate = 100.0f;

// Flag to start/stop sampling
bool samplingActive = false;

// Output buffer
char outputBuffer[128];

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("MAX30102 Filtered Signal Test with Adjustable Filters");
  Serial.println("---------------------------------------------------");

  // Initialize I2C
  Wire.begin();

  // Configure sensor with specific settings
  config.sensorMode = MODE_SPO2;       // Use SpO2 mode (RED + IR LEDs)
  config.sampleRate = SAMPLERATE_400;  // 800 samples per second
  config.pulseWidth = PULSEWIDTH_69;   // 69μs pulse width
  config.adcRange = ADCRANGE_16384;    // Full range
  config.sampleAvg = SAMPLEAVG_4;      // Average 4 samples
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
  printSensorConfig();

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
  
  // Calculate sample interval in ms
  sampleInterval = 1000 / (unsigned long)actualSampleRate;
  
  Serial.println("\nReady to stream filtered signal data. Send commands:");
  Serial.println("  's' - Start/stop sampling");
  Serial.println("  '+' - Increase LED power");
  Serial.println("  '-' - Decrease LED power");
  Serial.println("  'r' - Reset sensor");
  Serial.println("  'i' - Sensor info");
  Serial.println("\nFilter adjustment commands:");
  Serial.println("  'a' - Adjust breathing filter low cutoff");
  Serial.println("  'b' - Adjust breathing filter high cutoff");
  Serial.println("  'c' - Adjust heart rate filter low cutoff");
  Serial.println("  'd' - Adjust heart rate filter high cutoff");
  Serial.println("  'f' - Show current filter settings");
  
  // Print CSV header
  Serial.println("\nOutputting CSV format:");
  Serial.println("RawIR,BreathingFilter,HeartRateFilter");

  // Clear any unread data
  sensor.clearFIFO();

  // Start with sampling disabled
  samplingActive = false;
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // Read and process sensor data if active
  if (samplingActive && (millis() - lastSampleTime >= sampleInterval)) {
    lastSampleTime = millis();

    // Read data from sensor
    uint32_t redValue, irValue;

    if (sensor.available() > 0) {
      if (sensor.getRawData(&redValue, &irValue)) {
        // Apply filters to IR signal (typically cleaner than RED for PPG)
        float irFloat = (float)irValue;
        float irFiltered_BR = filters.applyFilter(irFloat, breathingFilter, breathingFilterState);
        float irFiltered_HR = filters.applyFilter(irFloat, heartRateFilter, heartRateFilterState);
        
        // Output all values in CSV format
        snprintf(outputBuffer, sizeof(outputBuffer), "%.2f,%.2f", 
                 irFiltered_BR, irFiltered_HR);
        Serial.println(outputBuffer);
      }
    }
  }
}

// Design filters with current parameters
void designFilters() {
  // Design breathing band-pass filter (0.1-0.5 Hz typical for respiration)
  breathingFilter = filters.designBandPassFilter(breathingLowCutoff, breathingHighCutoff);
  
  // Design heart rate band-pass filter (0.5-5 Hz typical for heart rate)
  heartRateFilter = filters.designBandPassFilter(heartRateLowCutoff, heartRateHighCutoff);
  
  // Print current filter settings
  Serial.println("\nCurrent Filter Settings:");
  Serial.print("Breathing band-pass: "); 
  Serial.print(breathingLowCutoff); Serial.print(" - "); 
  Serial.print(breathingHighCutoff); Serial.println(" Hz");
  
  Serial.print("Heart rate band-pass: "); 
  Serial.print(heartRateLowCutoff); Serial.print(" - "); 
  Serial.print(heartRateHighCutoff); Serial.println(" Hz");
}

// Reset filter states when parameters change
void resetFilterStates() {
  for (int i = 0; i < 4; i++) {
    breathingFilterState[i] = 0.0f;
    heartRateFilterState[i] = 0.0f;
  }
}

void printSensorConfig() {
  Serial.println("\nSensor Configuration:");
  Serial.print("Mode: ");
  switch (config.sensorMode) {
    case MODE_HEARTRATE: Serial.println("Heart Rate (RED LED)"); break;
    case MODE_SPO2: Serial.println("SpO2 (RED + IR LEDs)"); break;
    case MODE_MULTILED: Serial.println("Multi-LED (RED + IR + GREEN LEDs)"); break;
    default: Serial.println("Unknown"); break;
  }

  Serial.print("Sample Rate: ");
  switch (config.sampleRate) {
    case SAMPLERATE_50: Serial.println("50 Hz"); break;
    case SAMPLERATE_100: Serial.println("100 Hz"); break;
    case SAMPLERATE_200: Serial.println("200 Hz"); break;
    case SAMPLERATE_400: Serial.println("400 Hz"); break;
    case SAMPLERATE_800: Serial.println("800 Hz"); break;
    case SAMPLERATE_1000: Serial.println("1000 Hz"); break;
    case SAMPLERATE_1600: Serial.println("1600 Hz"); break;
    case SAMPLERATE_3200: Serial.println("3200 Hz"); break;
    default: Serial.println("Unknown"); break;
  }

  Serial.print("Pulse Width: ");
  switch (config.pulseWidth) {
    case PULSEWIDTH_69: Serial.println("69 μs (15-bit)"); break;
    case PULSEWIDTH_118: Serial.println("118 μs (16-bit)"); break;
    case PULSEWIDTH_215: Serial.println("215 μs (17-bit)"); break;
    case PULSEWIDTH_411: Serial.println("411 μs (18-bit)"); break;
    default: Serial.println("Unknown"); break;
  }

  Serial.print("RED LED Power: ");
  Serial.println(config.redLedAmplitude);

  Serial.print("IR LED Power: ");
  Serial.println(config.irLedAmplitude);
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 's': // Start/stop sampling
      samplingActive = !samplingActive;
      Serial.print("Sampling: ");
      Serial.println(samplingActive ? "STARTED" : "STOPPED");
      if (samplingActive) {
        sensor.clearFIFO(); // Clear old data
        
        // Reset filter states
        resetFilterStates();
        
        // Remind of CSV format
        Serial.println("RawIR,BreathingFilter,HeartRateFilter");
      }
      break;

    case '+': // Increase LED power
      if (config.redLedAmplitude < 255) {
        config.redLedAmplitude += 10;
        config.irLedAmplitude += 10;
        if (config.redLedAmplitude > 255) config.redLedAmplitude = 255;
        if (config.irLedAmplitude > 255) config.irLedAmplitude = 255;

        sensor.setLEDPulseAmplitude(1, config.redLedAmplitude);
        sensor.setLEDPulseAmplitude(2, config.irLedAmplitude);

        Serial.print("LED Power increased to: ");
        Serial.println(config.redLedAmplitude);
      } else {
        Serial.println("LED Power already at maximum");
      }
      break;

    case '-': // Decrease LED power
      if (config.redLedAmplitude > 10) {
        config.redLedAmplitude -= 10;
        config.irLedAmplitude -= 10;

        sensor.setLEDPulseAmplitude(1, config.redLedAmplitude);
        sensor.setLEDPulseAmplitude(2, config.irLedAmplitude);

        Serial.print("LED Power decreased to: ");
        Serial.println(config.redLedAmplitude);
      } else {
        Serial.println("LED Power already at minimum");
      }
      break;

    case 'r': // Reset sensor
      Serial.println("Resetting sensor...");
      sensor.softReset();
      delay(100);
      sensor.setup(config);
      sensor.clearFIFO();
      Serial.println("Sensor reset complete");
      break;

    case 'i': // Sensor info
      Serial.println("\nSensor Information:");
      Serial.print("Part ID: 0x");
      Serial.println(sensor.readPartID(), HEX);

      Serial.print("Temperature: ");
      Serial.print(sensor.getTemperature(), 2);
      Serial.println(" °C");

      Serial.print("Samples in FIFO: ");
      Serial.println(sensor.available());
      break;
      
    case 'a': // Change breathing filter low cutoff
      adjustBreathingLowCutoff();
      break;
      
    case 'b': // Change breathing filter high cutoff
      adjustBreathingHighCutoff();
      break;
      
    case 'c': // Change heart rate filter low cutoff
      adjustHeartRateLowCutoff();
      break;
      
    case 'd': // Change heart rate filter high cutoff
      adjustHeartRateHighCutoff();
      break;
      
    case 'f': // Show current filter settings
      designFilters(); // This will print the current settings
      break;

    default:
      // Ignore unrecognized commands
      break;
  }
}

// Function to adjust breathing filter low cutoff frequency
void adjustBreathingLowCutoff() {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }
  
  Serial.println("\nAdjust Breathing Filter Low Cutoff:");
  Serial.println("Current range: " + String(breathingLowCutoff) + " - " + String(breathingHighCutoff) + " Hz");
  Serial.println("Enter new low cutoff value (0.05-" + String(breathingHighCutoff - 0.05) + " Hz):");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();
  
  // Validate input
  if (newCutoff >= 0.05 && newCutoff < breathingHighCutoff) {
    breathingLowCutoff = newCutoff;
    
    // Redesign filter with new cutoff
    breathingFilter = filters.designBandPassFilter(breathingLowCutoff, breathingHighCutoff);
    
    // Reset filter states
    resetFilterStates();
    
    Serial.println("Breathing filter low cutoff set to: " + String(breathingLowCutoff) + " Hz");
  } else {
    Serial.println("Invalid value. Keeping current setting.");
  }
  
  // Resume sampling if it was active
  if (wasActive) {
    samplingActive = true;
    Serial.println("Sampling resumed");
    Serial.println("RawIR,BreathingFilter,HeartRateFilter");
  }
}

// Function to adjust breathing filter high cutoff frequency
void adjustBreathingHighCutoff() {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }
  
  Serial.println("\nAdjust Breathing Filter High Cutoff:");
  Serial.println("Current range: " + String(breathingLowCutoff) + " - " + String(breathingHighCutoff) + " Hz");
  Serial.println("Enter new high cutoff value (" + String(breathingLowCutoff + 0.05) + "-" + String(heartRateLowCutoff) + " Hz):");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();
  
  // Validate input (make sure it's higher than the low cutoff but not overlapping heart rate)
  if (newCutoff > breathingLowCutoff && newCutoff <= heartRateLowCutoff) {
    breathingHighCutoff = newCutoff;
    
    // Redesign filter with new cutoff
    breathingFilter = filters.designBandPassFilter(breathingLowCutoff, breathingHighCutoff);
    
    // Reset filter states
    resetFilterStates();
    
    Serial.println("Breathing filter high cutoff set to: " + String(breathingHighCutoff) + " Hz");
  } else {
    Serial.println("Invalid value. Keeping current setting.");
  }
  
  // Resume sampling if it was active
  if (wasActive) {
    samplingActive = true;
    Serial.println("Sampling resumed");
    Serial.println("RawIR,BreathingFilter,HeartRateFilter");
  }
}

// Function to adjust heart rate filter low cutoff frequency
void adjustHeartRateLowCutoff() {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }
  
  Serial.println("\nAdjust Heart Rate Filter Low Cutoff:");
  Serial.println("Current range: " + String(heartRateLowCutoff) + " - " + String(heartRateHighCutoff) + " Hz");
  Serial.println("Enter new low cutoff value (" + String(breathingHighCutoff) + "-" + String(heartRateHighCutoff - 0.05) + " Hz):");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();
  
  // Validate input
  if (newCutoff >= breathingHighCutoff && newCutoff < heartRateHighCutoff) {
    heartRateLowCutoff = newCutoff;
    
    // Redesign filter with new cutoff
    heartRateFilter = filters.designBandPassFilter(heartRateLowCutoff, heartRateHighCutoff);
    
    // Reset filter states
    resetFilterStates();
    
    Serial.println("Heart rate filter low cutoff set to: " + String(heartRateLowCutoff) + " Hz");
  } else {
    Serial.println("Invalid value. Keeping current setting.");
  }
  
  // Resume sampling if it was active
  if (wasActive) {
    samplingActive = true;
    Serial.println("Sampling resumed");
    Serial.println("RawIR,BreathingFilter,HeartRateFilter");
  }
}

// Function to adjust heart rate filter high cutoff frequency
void adjustHeartRateHighCutoff() {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }
  
  Serial.println("\nAdjust Heart Rate Filter High Cutoff:");
  Serial.println("Current range: " + String(heartRateLowCutoff) + " - " + String(heartRateHighCutoff) + " Hz");
  Serial.println("Enter new high cutoff value (" + String(heartRateLowCutoff + 0.05) + "-20 Hz):");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();
  
  // Validate input
  if (newCutoff > heartRateLowCutoff && newCutoff <= 20.0) {
    heartRateHighCutoff = newCutoff;
    
    // Redesign filter with new cutoff
    heartRateFilter = filters.designBandPassFilter(heartRateLowCutoff, heartRateHighCutoff);
    
    // Reset filter states
    resetFilterStates();
    
    Serial.println("Heart rate filter high cutoff set to: " + String(heartRateHighCutoff) + " Hz");
  } else {
    Serial.println("Invalid value. Keeping current setting.");
  }
  
  // Resume sampling if it was active
  if (wasActive) {
    samplingActive = true;
    Serial.println("Sampling resumed");
    Serial.println("RawIR,BreathingFilter,HeartRateFilter");
  }
}