# PulseOx Firmware

A Rust firmware for reading raw IR and RED LED values from a MAX30102 pulse oximeter sensor using a Teensy 4.1 microcontroller. This project is built using the [teensy4-rs](https://github.com/mciantyre/teensy4-rs) framework and RTIC (Real-Time Interrupt-driven Concurrency) for better task management.

## Hardware Requirements

- Teensy 4.1 board
- MAX30102 pulse oximeter and heart-rate sensor
- I2C connections (pins 16/17 on Teensy 4.1)
- USB connection for serial output and programming

## Wiring

Connect the MAX30102 sensor to the Teensy 4.1 as follows:

| MAX30102 | Teensy 4.1 |
|----------|------------|
| VIN      | 3.3V       |
| GND      | GND        |
| SCL      | Pin 16     |
| SDA      | Pin 17     |
| INT      | Not used   |

## Building and Flashing

1. Install the required tools:
   ```
   rustup target add thumbv7em-none-eabihf
   cargo install cargo-binutils
   ```

2. Build the firmware:
   ```
   cargo build --release
   ```

3. Convert to HEX file:
   ```
   cargo objcopy --release -- -O ihex pulse-ox-firmware.hex
   ```

4. Flash to Teensy 4.1:
   - Press the button on your Teensy to enter programming mode
   - Use the Teensy Loader Application or `teensy_loader_cli`:
     ```
     teensy_loader_cli --mcu=imxrt1062 -w pulse-ox-firmware.hex
     ```

Note: Make sure you have the correct version of the Teensy Loader that supports the Teensy 4.1.

## Usage

After flashing, the firmware will:
1. Initialize the USB logging system
2. Setup the MAX30102 sensor
3. Configure it for heart rate reading mode with the following settings:
   - 8 sample averaging
   - LED pulse amplitude: 30
   - LED pulse width: 411μs
   - Sampling rate: 100Hz
   - FIFO rollover enabled
4. Use RTIC to schedule periodic sensor readings (every 1 second)
5. Output raw RED and IR values over USB serial

Connect to the Teensy's USB serial port to view the readings. The logging system will output properly formatted log messages with timestamps.

## Troubleshooting

If you encounter issues:

1. Check the I2C connections between the MAX30102 and Teensy 4.1
2. Verify that the MAX30102 sensor is powered correctly (3.3V)
3. Make sure the serial monitor is configured at 115200 baud
4. Try reducing the I2C clock speed if communication is unreliable
