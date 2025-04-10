#![no_std]
#![no_main]

use panic_halt as _;
use max3010x::{Max3010x, Led, SampleAveraging, SamplingRate, LedPulseWidth};
use embedded_hal::delay::DelayNs;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Set up the serial port for debugging
    let mut serial = arduino_hal::default_serial!(dp, pins, 9600);

    // Set up I2C for the sensor
    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(), // SDA
        pins.a5.into_pull_up_input(), // SCL
        50000,
    );

    // Set up the MAX3010x sensor
    let mut delay = arduino_hal::Delay::new();

    // Write directly to serial using individual bytes
    for b in b"Initializing MAX30102 sensor...\r\n" {
        serial.write_byte(*b);
    }

    // Create the sensor instance
    let sensor = Max3010x::new_max30102(i2c);

    // Switch to heart-rate mode directly
    let heart_rate_result = sensor.into_heart_rate();
    match heart_rate_result {
        Ok(mut hr_sensor) => {
            for b in b"Switched to heart rate mode\r\n" {
                serial.write_byte(*b);
            }

            // Configure the sensor
            if hr_sensor.set_sample_averaging(SampleAveraging::Sa8).is_err() {
                for b in b"Failed to set sample averaging\r\n" {
                    serial.write_byte(*b);
                }
            }

            // Set higher pulse amplitude for better readings
            if hr_sensor.set_pulse_amplitude(Led::All, 30).is_err() {
                for b in b"Failed to set pulse amplitude\r\n" {
                    serial.write_byte(*b);
                }
            }

            if hr_sensor.set_pulse_width(LedPulseWidth::Pw411).is_err() {
                for b in b"Failed to set pulse width\r\n" {
                    serial.write_byte(*b);
                }
            }

            if hr_sensor.set_sampling_rate(SamplingRate::Sps100).is_err() {
                for b in b"Failed to set sampling rate\r\n" {
                    serial.write_byte(*b);
                }
            }

            if hr_sensor.enable_fifo_rollover().is_err() {
                for b in b"Failed to enable FIFO rollover\r\n" {
                    serial.write_byte(*b);
                }
            }

            for b in b"Starting sensor readings...\r\n" {
                serial.write_byte(*b);
            }

            // Status LED
            let mut led = pins.d13.into_output();

            // Buffer for readings - adjust size based on available memory
            let mut data = [0; 32];

            loop {
                led.toggle();

                // Read data from the sensor
                match hr_sensor.read_fifo(&mut data) {
                    Ok(samples_read) => {
                        if samples_read > 0 {
                            // Output format: "DATA: RED IR"
                            for b in b"DATA: " {
                                serial.write_byte(*b);
                            }

                            // Return raw data from both LEDs
                            // In heart rate mode, even indices are RED, odd indices are IR
                            for i in 0..samples_read.min(16) {  // Limit to 16 samples per read
                                // Convert the number to ASCII digits
                                // Cast i to usize for array indexing
                                let value = data[i as usize];

                                // Simple number formatting
                                if value > 9999 {
                                    serial.write_byte(b'0' + ((value / 10000) % 10) as u8);
                                }
                                if value > 999 {
                                    serial.write_byte(b'0' + ((value / 1000) % 10) as u8);
                                }
                                if value > 99 {
                                    serial.write_byte(b'0' + ((value / 100) % 10) as u8);
                                }
                                if value > 9 {
                                    serial.write_byte(b'0' + ((value / 10) % 10) as u8);
                                }
                                serial.write_byte(b'0' + (value % 10) as u8);

                                // Identify whether it's RED or IR
                                if i % 2 == 0 {
                                    for b in b" R, " {
                                        serial.write_byte(*b);
                                    }
                                } else {
                                    for b in b" I; " {
                                        serial.write_byte(*b);
                                    }
                                }
                            }

                            // End the line
                            for b in b"\r\n" {
                                serial.write_byte(*b);
                            }
                        } else {
                            for b in b"No samples available\r\n" {
                                serial.write_byte(*b);
                            }
                        }
                    },
                    Err(_) => {
                        for b in b"Failed to read from sensor\r\n" {
                            serial.write_byte(*b);
                        }
                    }
                }

                // Use the correct delay for embedded-hal 1.0.0
                delay.delay_ns(1_000_000_000); // 1 second
            }
        },
        Err(_) => {
            for b in b"Failed to switch to heart rate mode\r\n" {
                serial.write_byte(*b);
            }
            loop { }
        }
    }
}
