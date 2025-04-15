// MAX30102_Handler.h
#ifndef MAX30102_HANDLER_H
#define MAX30102_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "core.h"

// MAX30102 Register Map
#define MAX30102_INTSTAT1     0x00
#define MAX30102_INTSTAT2     0x01
#define MAX30102_INTENABLE1   0x02
#define MAX30102_INTENABLE2   0x03
#define MAX30102_FIFOWRITEPTR 0x04
#define MAX30102_FIFOOVERFLOW 0x05
#define MAX30102_FIFOREADPTR  0x06
#define MAX30102_FIFODATA     0x07
#define MAX30102_FIFOCONFIG   0x08
#define MAX30102_MODECONFIG   0x09
#define MAX30102_SPO2CONFIG   0x0A
#define MAX30102_LED1_PULSEAMP 0x0C
#define MAX30102_LED2_PULSEAMP 0x0D
#define MAX30102_LED3_PULSEAMP 0x0E
#define MAX30102_PROXINTTHRESH 0x10
#define MAX30102_REV_ID       0xFE
#define MAX30102_PART_ID      0xFF

// Modes (for MODE_CONFIG)
#define MAX30102_MODE_HRONLY    0x02
#define MAX30102_MODE_SPO2      0x03
#define MAX30102_MODE_MULTILED  0x07

// Pulse Width (uS)
#define MAX30102_PULSEWIDTH_69  0x00 //  69 uS
#define MAX30102_PULSEWIDTH_118 0x01 // 118 uS
#define MAX30102_PULSEWIDTH_215 0x02 // 215 uS
#define MAX30102_PULSEWIDTH_411 0x03 // 411 uS

// Sample Rate (Hz)
#define MAX30102_SAMPLERATE_50    0x00
#define MAX30102_SAMPLERATE_100   0x01
#define MAX30102_SAMPLERATE_200   0x02
#define MAX30102_SAMPLERATE_400   0x03
#define MAX30102_SAMPLERATE_800   0x04
#define MAX30102_SAMPLERATE_1000  0x05
#define MAX30102_SAMPLERATE_1600  0x06
#define MAX30102_SAMPLERATE_3200  0x07

// ADC Range
#define MAX30102_ADCRANGE_2048  0x00
#define MAX30102_ADCRANGE_4096  0x01
#define MAX30102_ADCRANGE_8192  0x02
#define MAX30102_ADCRANGE_16384 0x03

// I²C Address
#define MAX30102_I2C_ADDRESS 0x57

// FIFO sample averages
#define MAX30102_SAMPLEAVG_1     0x00
#define MAX30102_SAMPLEAVG_2     0x01
#define MAX30102_SAMPLEAVG_4     0x02
#define MAX30102_SAMPLEAVG_8     0x03
#define MAX30102_SAMPLEAVG_16    0x04
#define MAX30102_SAMPLEAVG_32    0x05

class MAX30102_Handler {
public:
    MAX30102_Handler(TwoWire &wirePort = Wire);

    // Sensor status
    bool begin();
    bool isConnected();
    uint8_t readPartID();

    // Configuration
    void setup(PulseOxConfig &config);
    void setMode(uint8_t mode);
    void setSampleRate(uint8_t sampleRate);
    void setPulseWidth(uint8_t pulseWidth);
    void setLEDPulseAmplitude(uint8_t led, uint8_t amplitude);
    void setSampleAverage(uint8_t samples);
    void setADCRange(uint8_t adcRange);
    void setFIFOAlmostFull(uint8_t samples);

    // Data acquisition
    void clearFIFO();
    uint16_t check();
    uint8_t available();
    bool getRawData(uint32_t *red, uint32_t *ir);

    // Direct register access
    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    // Sensor information
    float getTemperature();
    void shutDown();
    void wakeUp();
    void softReset();

private:
    TwoWire &_i2cPort;
    uint8_t _readPointer;
    uint8_t _writePointer;

    uint32_t readFIFO();
    uint8_t getReadPointer();
    uint8_t getWritePointer();
};

#endif // MAX30102_HANDLER_H
