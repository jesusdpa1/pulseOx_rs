// core.h
#ifndef CORE_H
#define CORE_H

#include <Arduino.h>

// Sample averaging options
typedef enum {
  SAMPLEAVG_1  = 0x00,
  SAMPLEAVG_2  = 0x01,
  SAMPLEAVG_4  = 0x02,
  SAMPLEAVG_8  = 0x03,
  SAMPLEAVG_16 = 0x04,
  SAMPLEAVG_32 = 0x05
} SampleAveraging;

// LED pulse width options (affects ADC resolution)
typedef enum {
  PULSEWIDTH_69  = 0x00, // 69us, 15-bit resolution
  PULSEWIDTH_118 = 0x01, // 118us, 16-bit resolution
  PULSEWIDTH_215 = 0x02, // 215us, 17-bit resolution
  PULSEWIDTH_411 = 0x03  // 411us, 18-bit resolution
} PulseWidth;

// Sample rate options
typedef enum {
  SAMPLERATE_50   = 0x00, // 50 samples per second
  SAMPLERATE_100  = 0x01, // 100 samples per second
  SAMPLERATE_200  = 0x02, // 200 samples per second
  SAMPLERATE_400  = 0x03, // 400 samples per second
  SAMPLERATE_800  = 0x04, // 800 samples per second
  SAMPLERATE_1000 = 0x05, // 1000 samples per second
  SAMPLERATE_1600 = 0x06, // 1600 samples per second
  SAMPLERATE_3200 = 0x07  // 3200 samples per second
} SampleRate;

// ADC range options
typedef enum {
  ADCRANGE_2048  = 0x00, // ADC range 0 - 2048 nA
  ADCRANGE_4096  = 0x01, // ADC range 0 - 4096 nA
  ADCRANGE_8192  = 0x02, // ADC range 0 - 8192 nA
  ADCRANGE_16384 = 0x03  // ADC range 0 - 16384 nA
} AdcRange;

// Sensor mode
typedef enum {
  MODE_HEARTRATE = 0x02, // Heart Rate mode (RED LED only)
  MODE_SPO2      = 0x03, // SpO2 mode (RED and IR LEDs)
  MODE_MULTILED  = 0x07  // Multi-LED mode (RED, IR, and GREEN if available)
} SensorMode;

// Configuration settings structure
struct PulseOxConfig {
    // MAX30102 specific settings
    SensorMode sensorMode;           // Operating mode
    SampleRate sampleRate;           // Number of samples per second
    SampleAveraging sampleAvg;       // Number of samples to average
    PulseWidth pulseWidth;           // LED pulse width
    AdcRange adcRange;               // ADC range
    uint8_t redLedAmplitude;         // Red LED current (0-255)
    uint8_t irLedAmplitude;          // IR LED current (0-255)
    uint8_t greenLedAmplitude;       // Green LED current (0-255, if available)
    uint8_t fifoAlmostFull;          // FIFO almost full value (0-31)
    bool redLedEnabled;              // Enable/disable red LED channel
    bool irLedEnabled;               // Enable/disable IR LED channel
    bool greenLedEnabled;            // Enable/disable green LED channel (if available)

    // Default constructor with reasonable defaults
    PulseOxConfig() :
        sensorMode(MODE_SPO2),
        sampleRate(SAMPLERATE_100),
        sampleAvg(SAMPLEAVG_8),
        pulseWidth(PULSEWIDTH_411),
        adcRange(ADCRANGE_16384),
        redLedAmplitude(60),
        irLedAmplitude(60),
        greenLedAmplitude(60),
        fifoAlmostFull(15),
        redLedEnabled(true),
        irLedEnabled(true),
        greenLedEnabled(false) {}
};

#endif // CORE_H
