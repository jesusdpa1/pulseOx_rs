#![no_std]
#![no_main]

use panic_halt as _;
use max3010x::{Max3010x, Led, SampleAveraging, SamplingRate, PulseWidth, LedPowerLevel};
use embedded_hal::blocking::delay::DelayMs;
use core::fmt::Write;

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
    let mut max3010x = Max3010x::new_max30105(i2c);
    
    // Initialize the sensor
    match max3010x.reset() {
        Ok(_) => writeln!(&mut serial, "Sensor reset successful").unwrap(),
        Err(_) => writeln!(&mut serial, "Failed to reset sensor").unwrap(),
    }
    
    delay.delay_ms(100u16);
    
    // Configure the sensor
    let config = max3010x.config()
        .led_mode(Led::All)
        .sampling_rate(SamplingRate::Sps100)
        .pulse_width(PulseWidth::Pw411)
        .sample_averaging(SampleAveraging::Sa8)
        .led_power_level(LedPowerLevel::Power27mA);
    
    match max3010x.set_config(config) {
        Ok(_) => writeln!(&mut serial, "Sensor configured successfully").unwrap(),
        Err(_) => writeln!(&mut serial, "Failed to configure sensor").unwrap(),
    }
    
    writeln!(&mut serial, "Starting sensor readings...").unwrap();
    
    // Status LED
    let mut led = pins.d13.into_output();
    
    loop {
        led.toggle();
        
        // Read data from the sensor
        match max3010x.read_all_samples() {
            Ok(samples) => {
                // For simplicity, just print the red LED readings
                // In a real application, you'd do SpO2 and heart rate calculations
                writeln!(&mut serial, "Red: {}, IR: {}, Green: {}", 
                    samples.red, samples.ir, samples.green.unwrap_or(0)).unwrap();
            },
            Err(_) => {
                writeln!(&mut serial, "Failed to read from sensor").unwrap();
            }
        }
        
        delay.delay_ms(1000u16);
    }
}