// Communication.cpp
#include "Communication.h"

Communication::Communication() {
}

void Communication::begin(unsigned long baudRate) {
  Serial.begin(baudRate);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("MAX30102 Filtered Signal Test with Enhanced Filters");
  Serial.println("------------------------------------------------------");
}

void Communication::handleSerialCommands(
  bool &samplingActive,
  PulseOxConfig &config,
  MAX30102_Handler &sensor,
  SignalFilters &filters,
  float &breathingLowCutoff,
  float &breathingHighCutoff,
  float &heartRateLowCutoff,
  float &heartRateHighCutoff
) {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 's': // Start/stop sampling
        samplingActive = !samplingActive;
        notifySamplingStatus(samplingActive);
        if (samplingActive) {
          sensor.clearFIFO(); // Clear old data

          // Remind of CSV format
          printCSVHeader();
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
        notifyReset();
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
        breathingLowCutoff = adjustBreathingLowCutoff(samplingActive,
          breathingLowCutoff, breathingHighCutoff);
        printFilterSettings(breathingLowCutoff, breathingHighCutoff,
          heartRateLowCutoff, heartRateHighCutoff);
        break;

      case 'b': // Change breathing filter high cutoff
        breathingHighCutoff = adjustBreathingHighCutoff(samplingActive,
          breathingLowCutoff, breathingHighCutoff, heartRateLowCutoff);
        printFilterSettings(breathingLowCutoff, breathingHighCutoff,
          heartRateLowCutoff, heartRateHighCutoff);
        break;

      case 'c': // Change heart rate filter low cutoff
        heartRateLowCutoff = adjustHeartRateLowCutoff(samplingActive,
          breathingHighCutoff, heartRateLowCutoff, heartRateHighCutoff);
        printFilterSettings(breathingLowCutoff, breathingHighCutoff,
          heartRateLowCutoff, heartRateHighCutoff);
        break;

      case 'd': // Change heart rate filter high cutoff
        heartRateHighCutoff = adjustHeartRateHighCutoff(samplingActive,
          heartRateLowCutoff, heartRateHighCutoff);
        printFilterSettings(breathingLowCutoff, breathingHighCutoff,
          heartRateLowCutoff, heartRateHighCutoff);
        break;

      case 'f': // Show current filter settings
        printFilterSettings(breathingLowCutoff, breathingHighCutoff,
          heartRateLowCutoff, heartRateHighCutoff);
        break;

      default:
        // Ignore unrecognized commands
        break;
    }
  }
}

void Communication::outputData(uint32_t irValue, float irFiltered_BR, float irFiltered_HR) {
  // Check for NaN values before printing
  if (isnan(irFiltered_BR)) irFiltered_BR = 0.0f;
  if (isnan(irFiltered_HR)) irFiltered_HR = 0.0f;

  // Output all values in CSV format
  snprintf(_outputBuffer, sizeof(_outputBuffer), "%lu,%.2f,%.2f",
           irValue, irFiltered_BR, irFiltered_HR);
  Serial.println(_outputBuffer);
}

void Communication::printSensorConfig(PulseOxConfig &config) {
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

void Communication::printFilterSettings(float breathingLowCutoff, float breathingHighCutoff,
                                      float heartRateLowCutoff, float heartRateHighCutoff) {
  Serial.println("\nCurrent Filter Settings:");
  Serial.print("Breathing band-pass: ");
  Serial.print(breathingLowCutoff); Serial.print(" - ");
  Serial.print(breathingHighCutoff); Serial.println(" Hz");

  Serial.print("Heart rate band-pass: ");
  Serial.print(heartRateLowCutoff); Serial.print(" - ");
  Serial.print(heartRateHighCutoff); Serial.println(" Hz");
}

void Communication::printMenu() {
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
}

void Communication::printCSVHeader() {
  Serial.println("\nOutputting CSV format:");
  Serial.println("RawIR,BreathingFilter,HeartRateFilter");
}

void Communication::notifyReset() {
  Serial.println("Sensor reset complete");
}

void Communication::notifySamplingStatus(bool active) {
  Serial.print("Sampling: ");
  Serial.println(active ? "STARTED" : "STOPPED");
}

float Communication::adjustBreathingLowCutoff(bool &samplingActive, float currentLow, float currentHigh) {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }

  Serial.println("\nAdjust Breathing Filter Low Cutoff:");
  Serial.println("Current range: " + String(currentLow) + " - " + String(currentHigh) + " Hz");
  Serial.println("Enter new low cutoff value (0.05-" + String(currentHigh - 0.05) + " Hz):");

  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }

  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();

  // Validate input
  if (newCutoff >= 0.05 && newCutoff < currentHigh) {
    Serial.println("Breathing filter low cutoff set to: " + String(newCutoff) + " Hz");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return newCutoff;
  } else {
    Serial.println("Invalid value. Keeping current setting.");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return currentLow;
  }
}

float Communication::adjustBreathingHighCutoff(bool &samplingActive, float currentLow, float currentHigh, float heartRateLow) {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }

  Serial.println("\nAdjust Breathing Filter High Cutoff:");
  Serial.println("Current range: " + String(currentLow) + " - " + String(currentHigh) + " Hz");
  Serial.println("Enter new high cutoff value (" + String(currentLow + 0.05) + "-" + String(heartRateLow) + " Hz):");

  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }

  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();

  // Validate input (make sure it's higher than the low cutoff but not overlapping heart rate)
  if (newCutoff > currentLow && newCutoff <= heartRateLow) {
    Serial.println("Breathing filter high cutoff set to: " + String(newCutoff) + " Hz");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return newCutoff;
  } else {
    Serial.println("Invalid value. Keeping current setting.");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return currentHigh;
  }
}

float Communication::adjustHeartRateLowCutoff(bool &samplingActive, float breathingHigh, float currentLow, float currentHigh) {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }

  Serial.println("\nAdjust Heart Rate Filter Low Cutoff:");
  Serial.println("Current range: " + String(currentLow) + " - " + String(currentHigh) + " Hz");
  Serial.println("Enter new low cutoff value (" + String(breathingHigh) + "-" + String(currentHigh - 0.05) + " Hz):");

  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }

  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();

  // Validate input
  if (newCutoff >= breathingHigh && newCutoff < currentHigh) {
    Serial.println("Heart rate filter low cutoff set to: " + String(newCutoff) + " Hz");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return newCutoff;
  } else {
    Serial.println("Invalid value. Keeping current setting.");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return currentLow;
  }
}

float Communication::adjustHeartRateHighCutoff(bool &samplingActive, float currentLow, float currentHigh) {
  bool wasActive = samplingActive;
  if (wasActive) {
    samplingActive = false; // Pause sampling
    Serial.println("Sampling paused for filter adjustment");
  }

  Serial.println("\nAdjust Heart Rate Filter High Cutoff:");
  Serial.println("Current range: " + String(currentLow) + " - " + String(currentHigh) + " Hz");
  Serial.println("Enter new high cutoff value (" + String(currentLow + 0.05) + "-20 Hz):");

  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }

  // Read the new value
  String input = Serial.readStringUntil('\n');
  float newCutoff = input.toFloat();

  // Validate input
  if (newCutoff > currentLow && newCutoff <= 20.0) {
    Serial.println("Heart rate filter high cutoff set to: " + String(newCutoff) + " Hz");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return newCutoff;
  } else {
    Serial.println("Invalid value. Keeping current setting.");

    // Resume sampling if it was active
    if (wasActive) {
      samplingActive = true;
      Serial.println("Sampling resumed");
      printCSVHeader();
    }

    return currentHigh;
  }
}
