#![no_std]
#![no_main]

// Only import what we need in the module-level code
use teensy4_panic as _; // Panic handler

// Import RTIC
#[rtic::app(device = teensy4_bsp)]
mod app {
    use max3010x::{Led, LedPulseWidth, Max3010x, SampleAveraging, SamplingRate};
    use teensy4_bsp::{self, board};
    use rtic_monotonics::systick::*;
    use log::{info, error};

    // Define shared resources
    #[shared]
    struct Shared {
        // Resources that can be accessed from multiple tasks
    }

    // Define local resources
    #[local]
    struct Local {
        sensor: max3010x::Max3010x<
            imxrt_hal::i2c::I2c<imxrt_hal::iomuxc::consts::U1>,
            max3010x::marker::ic::Max30102,
            max3010x::marker::mode::HeartRate
        >,
        data_buffer: [u32; 32],
    }

    // Initialization function
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Setup logging
        let logging = imxrt_log::Poller;
        logging.initialize(Default::default()).unwrap();

        info!("Starting PulseOx firmware...");

        // Start the monotonic timer
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 600_000_000, systick_token);

        // Get board resources
        let board::Resources {
            mut pins,
            mut i2c1,
            ..
        } = board::t41(ctx.device);

        // Initialize I2C pins for MAX30102
        i2c1.set_pins(pins.p16, pins.p17);
        i2c1.set_clock_rate(400_000);

        info!("Initializing MAX30102 sensor...");

        // Create the MAX30102 sensor instance
        let sensor = match Max3010x::new_max30102(i2c1) {
            Ok(s) => s,
            Err(_) => {
                error!("Failed to initialize sensor! Check connections");
                panic!("Sensor initialization failed");
            }
        };

        // Configure the sensor for heart rate mode
        let mut sensor = match sensor.into_heart_rate() {
            Ok(s) => s,
            Err(_) => {
                error!("Failed to set heart rate mode!");
                panic!("Sensor mode configuration failed");
            }
        };

        // Configure the sensor parameters
        if let Err(_) = sensor.set_sample_averaging(SampleAveraging::Sa8) {
            error!("Failed to set sample averaging");
        }

        if let Err(_) = sensor.set_pulse_amplitude(Led::All, 30) {
            error!("Failed to set pulse amplitude");
        }

        if let Err(_) = sensor.set_pulse_width(LedPulseWidth::Pw411) {
            error!("Failed to set pulse width");
        }

        if let Err(_) = sensor.set_sampling_rate(SamplingRate::Sps100) {
            error!("Failed to set sampling rate");
        }

        if let Err(_) = sensor.enable_fifo_rollover() {
            error!("Failed to enable FIFO rollover");
        }

        info!("Sensor initialized successfully!");
        info!("Starting readings...");

        // Schedule the first sensor reading
        read_sensor::spawn().ok();

        // Initialize resources
        (
            Shared {
                // No shared resources yet
            },
            Local {
                sensor,
                data_buffer: [0; 32],
            },
        )
    }

    // Task that reads the sensor data
    #[task(local = [sensor, data_buffer])]
    async fn read_sensor(ctx: read_sensor::Context) {
        // Read data from the sensor
        match ctx.local.sensor.read_fifo(ctx.local.data_buffer) {
            Ok(samples_read) => {
                if samples_read > 0 {
                    // In heart rate mode, even indices are RED, odd indices are IR
                    for i in 0..samples_read.min(16) {
                        // Limit to 16 samples per read
                        if i % 2 == 0 {
                            info!("Red: {}", ctx.local.data_buffer[i]);
                        } else {
                            info!("IR: {}", ctx.local.data_buffer[i]);
                        }
                    }
                } else {
                    info!("No samples available");
                }
            }
            Err(_) => {
                error!("Failed to read from sensor");
            }
        }

        // Delay for 1 second before reading again
        Systick::delay(1.secs()).await;

        // Schedule next reading
        read_sensor::spawn().ok();
    }
}
