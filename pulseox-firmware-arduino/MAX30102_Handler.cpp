// MAX30102_Handler.cpp
#include "MAX30102_Handler.h"

MAX30102_Handler::MAX30102_Handler(TwoWire &wirePort) : _i2cPort(wirePort) {
    // Initialize variables
    _readPointer = 0;
    _writePointer = 0;
}

bool MAX30102_Handler::begin() {
    // Initialize I2C
    _i2cPort.begin();
    
    // Check if sensor is connected
    if (!isConnected())
        return false;
    
    // Reset the sensor
    softReset();
    
    // Reset FIFO pointers
    clearFIFO();
    
    return true;
}

bool MAX30102_Handler::isConnected() {
    _i2cPort.beginTransmission(MAX30102_I2C_ADDRESS);
    return (_i2cPort.endTransmission() == 0);
}

uint8_t MAX30102_Handler::readPartID() {
    return readRegister8(MAX30102_PART_ID);
}

void MAX30102_Handler::setup(PulseOxConfig &config) {
    // Configure sensor based on configuration
    softReset(); // Reset before configuration
    
    // Set mode
    setMode(config.sensorMode);
    
    // Set sample rate
    setSampleRate(config.sampleRate);
    
    // Set pulse width
    setPulseWidth(config.pulseWidth);
    
    // Set sample averaging
    setSampleAverage(config.sampleAvg);
    
    // Set ADC range
    setADCRange(config.adcRange);
    
    // Configure LED pulse amplitudes
    if (config.redLedEnabled)
        setLEDPulseAmplitude(1, config.redLedAmplitude);
    else
        setLEDPulseAmplitude(1, 0); // Turn off RED LED
        
    if (config.irLedEnabled)
        setLEDPulseAmplitude(2, config.irLedAmplitude);
    else
        setLEDPulseAmplitude(2, 0); // Turn off IR LED
        
    if (config.greenLedEnabled)
        setLEDPulseAmplitude(3, config.greenLedAmplitude);
    else
        setLEDPulseAmplitude(3, 0); // Turn off GREEN LED (if available)
    
    // Set FIFO almost full value
    setFIFOAlmostFull(config.fifoAlmostFull);
    
    // Clear FIFO before starting
    clearFIFO();
}

void MAX30102_Handler::setMode(uint8_t mode) {
    uint8_t currentMode = readRegister8(MAX30102_MODECONFIG);
    writeRegister8(MAX30102_MODECONFIG, (currentMode & 0xF8) | mode);
}

void MAX30102_Handler::setSampleRate(uint8_t sampleRate) {
    uint8_t currentSPO2 = readRegister8(MAX30102_SPO2CONFIG);
    writeRegister8(MAX30102_SPO2CONFIG, (currentSPO2 & 0xE3) | (sampleRate << 2));
}

void MAX30102_Handler::setPulseWidth(uint8_t pulseWidth) {
    uint8_t currentSPO2 = readRegister8(MAX30102_SPO2CONFIG);
    writeRegister8(MAX30102_SPO2CONFIG, (currentSPO2 & 0xFC) | pulseWidth);
}

void MAX30102_Handler::setLEDPulseAmplitude(uint8_t led, uint8_t amplitude) {
    switch(led) {
        case 1:
            writeRegister8(MAX30102_LED1_PULSEAMP, amplitude);
            break;
        case 2:
            writeRegister8(MAX30102_LED2_PULSEAMP, amplitude);
            break;
        case 3:
            writeRegister8(MAX30102_LED3_PULSEAMP, amplitude);
            break;
        default:
            // Invalid LED
            break;
    }
}

void MAX30102_Handler::setSampleAverage(uint8_t samples) {
    uint8_t currentFIFOConfig = readRegister8(MAX30102_FIFOCONFIG);
    writeRegister8(MAX30102_FIFOCONFIG, (currentFIFOConfig & 0x1F) | (samples << 5));
}

void MAX30102_Handler::setADCRange(uint8_t adcRange) {
    uint8_t currentSPO2 = readRegister8(MAX30102_SPO2CONFIG);
    writeRegister8(MAX30102_SPO2CONFIG, (currentSPO2 & 0x9F) | (adcRange << 5));
}

void MAX30102_Handler::setFIFOAlmostFull(uint8_t samples) {
    // The almost full value is 32 - threshold
    writeRegister8(MAX30102_FIFOCONFIG, readRegister8(MAX30102_FIFOCONFIG) & 0xF0 | (32 - samples));
}

void MAX30102_Handler::clearFIFO() {
    writeRegister8(MAX30102_FIFOWRITEPTR, 0);
    writeRegister8(MAX30102_FIFOOVERFLOW, 0);
    writeRegister8(MAX30102_FIFOREADPTR, 0);
}

uint16_t MAX30102_Handler::check() {
    // Check how many samples are available in the FIFO
    // Returns the number of samples available
    uint8_t wrPtr = getWritePointer();
    uint8_t rdPtr = getReadPointer();
    
    int numberOfSamples = wrPtr - rdPtr;
    if (numberOfSamples < 0)
        numberOfSamples += 32; // Wrap condition
    
    return numberOfSamples;
}

uint8_t MAX30102_Handler::available() {
    return check();
}

bool MAX30102_Handler::getRawData(uint32_t *red, uint32_t *ir) {
    // Read data from FIFO
    if (available() == 0)
        return false;
    
    *red = readFIFO();
    *ir = readFIFO();
    
    return true;
}

uint32_t MAX30102_Handler::readFIFO() {
    uint32_t value = 0;
    
    // Reading 3 bytes (24 bits) from FIFO
    _i2cPort.beginTransmission(MAX30102_I2C_ADDRESS);
    _i2cPort.write(MAX30102_FIFODATA);
    _i2cPort.endTransmission(false);
    
    _i2cPort.requestFrom(MAX30102_I2C_ADDRESS, 3);
    
    // MSB first
    value = _i2cPort.read() << 16;
    value |= _i2cPort.read() << 8;
    value |= _i2cPort.read();
    
    return value;
}

uint8_t MAX30102_Handler::getReadPointer() {
    return readRegister8(MAX30102_FIFOREADPTR);
}

uint8_t MAX30102_Handler::getWritePointer() {
    return readRegister8(MAX30102_FIFOWRITEPTR);
}

float MAX30102_Handler::getTemperature() {
    // The temp register is updated when the TEMP_EN bit is set in mode configuration
    
    // Step 1: Set TEMP_EN bit
    uint8_t modeConfig = readRegister8(MAX30102_MODECONFIG);
    writeRegister8(MAX30102_MODECONFIG, modeConfig | 0x08); // Set TEMP_EN bit
    
    // Step 2: Wait for temperature conversion
    delay(100);
    
    // Step 3: Read die temperature registers
    int8_t tempInt = (int8_t)readRegister8(0x1F); // Integer part
    uint8_t tempFrac = readRegister8(0x20); // Fractional part (0.0625°C resolution)
    
    // Step 4: Clear TEMP_EN bit
    modeConfig = readRegister8(MAX30102_MODECONFIG);
    writeRegister8(MAX30102_MODECONFIG, modeConfig & ~0x08); // Clear TEMP_EN bit
    
    // Calculate temperature in °C
    return tempInt + (tempFrac * 0.0625);
}

void MAX30102_Handler::shutDown() {
    // Set the SHUTDOWN bit
    uint8_t modeConfig = readRegister8(MAX30102_MODECONFIG);
    writeRegister8(MAX30102_MODECONFIG, modeConfig | 0x80);
}

void MAX30102_Handler::wakeUp() {
    // Clear the SHUTDOWN bit
    uint8_t modeConfig = readRegister8(MAX30102_MODECONFIG);
    writeRegister8(MAX30102_MODECONFIG, modeConfig & ~0x80);
}

void MAX30102_Handler::softReset() {
    // Set the RESET bit
    writeRegister8(MAX30102_MODECONFIG, 0x40);
    
    // Wait for the reset to complete
    delay(50);
}

uint8_t MAX30102_Handler::readRegister8(uint8_t reg) {
    _i2cPort.beginTransmission(MAX30102_I2C_ADDRESS);
    _i2cPort.write(reg);
    _i2cPort.endTransmission(false);
    
    _i2cPort.requestFrom(MAX30102_I2C_ADDRESS, 1);
    return _i2cPort.read();
}

void MAX30102_Handler::writeRegister8(uint8_t reg, uint8_t value) {
    _i2cPort.beginTransmission(MAX30102_I2C_ADDRESS);
    _i2cPort.write(reg);
    _i2cPort.write(value);
    _i2cPort.endTransmission();
}