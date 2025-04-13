#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [LPUART8])]
mod app {
    use teensy4_bsp as bsp;
    use bsp::board;
    use bsp::hal::lpuart;
    use cortex_m::asm;
    use rtic_monotonics::systick::*;
    use fugit::ExtU32;

    use max3010x::{
        Led,
        LedPulseWidth,
        Max3010x,
        SampleAveraging,
        SamplingRate,
        // AdcRange,
    };

    #[shared]
    struct Shared {
        lpuart2: board::Lpuart2,
    }

    #[local]
    struct Local {
        led: board::Led,
        sensor: max3010x::Max3010x<
            board::Lpi2c1,
            max3010x::marker::ic::Max30102,
            max3010x::marker::mode::HeartRate
        >,
        tx_buffer: [u32; 32],
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 600_000_000, systick_token);

        let board::Resources {
            mut pins,
            mut gpio2,
            lpuart2,
            lpi2c1,
            ..
        } = board::t41(cx.device);

        let led = board::led(&mut gpio2, pins.p13);
        led.set();

        let mut lpuart2 = board::lpuart(lpuart2, pins.p14, pins.p15, 115200);
        lpuart2.disable(|lpuart2| {
            lpuart2.disable_fifo(lpuart::Direction::Tx);
            lpuart2.disable_fifo(lpuart::Direction::Rx);
            lpuart2.set_interrupts(lpuart::Interrupts::RECEIVE_FULL);
            lpuart2.set_parity(None);
        });

        let init_msg = b"PulseOx Sensor Initializing...\r\n";
        for &byte in init_msg {
            while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
            lpuart2.write_byte(byte);
        }

        let i2c = board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz400);

        // Initialize sensor
        let sensor_result = Max3010x::new_max30102(i2c);

        // Switch to heart rate mode
        let mut sensor = match sensor_result.into_heart_rate() {
            Ok(s) => s,
            Err(_) => {
                let msg = b"Failed to enable heart rate mode\r\n";
                for &byte in msg {
                    while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                    lpuart2.write_byte(byte);
                }
                panic!("Failed to enable heart rate mode");
            }
        };

        // 💡 Sensor Configuration
        if sensor.set_pulse_amplitude(Led::All, 15).is_err()
            || sensor.set_pulse_width(LedPulseWidth::Pw411).is_err()
            || sensor.set_sample_averaging(SampleAveraging::Sa8).is_err()
            || sensor.set_sampling_rate(SamplingRate::Sps100).is_err()
        {
            let msg = b"Sensor config failed\r\n";
            for &byte in msg {
                while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                lpuart2.write_byte(byte);
            }
            panic!("Sensor config failed");
        } else {
            let msg = b"Sensor configured\r\n";
            for &byte in msg {
                while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                lpuart2.write_byte(byte);
            }
        }

        sensor_read::spawn().ok();

        (
            Shared { lpuart2 },
            Local {
                led,
                sensor,
                tx_buffer: [0; 32],
            }
        )
    }

    #[task(shared = [lpuart2], local = [sensor, led, tx_buffer], priority = 2)]
    async fn sensor_read(mut cx: sensor_read::Context) {
        let samples_read = read_sensor(cx.local.sensor, cx.local.tx_buffer);

        if samples_read > 0 {
            cx.shared.lpuart2.lock(|lpuart2| {
                let header = b"--- New Samples ---\r\n";
                for &byte in header {
                    while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                    lpuart2.write_byte(byte);
                }

                for i in 0..samples_read.min(16) as usize {
                    let value = cx.local.tx_buffer[i];
                    let led_type = if i % 2 == 0 { b"Red: " } else { b"IR:  " };

                    let mut num_buf = [0u8; 16];
                    let mut idx = 0;
                    let mut num = value;

                    if num == 0 {
                        num_buf[idx] = b'0';
                        idx += 1;
                    } else {
                        let mut digits = [0u8; 10];
                        let mut digit_count = 0;
                        while num > 0 {
                            digits[digit_count] = (num % 10) as u8 + b'0';
                            num /= 10;
                            digit_count += 1;
                        }
                        for j in 0..digit_count {
                            num_buf[idx] = digits[digit_count - 1 - j];
                            idx += 1;
                        }
                    }

                    num_buf[idx] = b'\r';
                    idx += 1;
                    num_buf[idx] = b'\n';
                    idx += 1;

                    for &byte in led_type {
                        while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                        lpuart2.write_byte(byte);
                    }

                    for &byte in &num_buf[0..idx] {
                        while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                        lpuart2.write_byte(byte);
                    }
                }
            });

            cx.local.led.toggle();
        } else {
            cx.shared.lpuart2.lock(|lpuart2| {
                let msg = b"No samples available\r\n";
                for &byte in msg {
                    while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                    lpuart2.write_byte(byte);
                }
            });
        }

        Systick::delay(1.secs()).await;
        sensor_read::spawn().ok();
    }

    fn read_sensor(
        sensor: &mut max3010x::Max3010x<
            board::Lpi2c1,
            max3010x::marker::ic::Max30102,
            max3010x::marker::mode::HeartRate,
        >,
        tx_buffer: &mut [u32; 32],
    ) -> usize {
        tx_buffer.fill(0);
        match sensor.read_fifo(tx_buffer) {
            Ok(samples_read) => samples_read as usize,
            Err(_) => 0,
        }
    }

    #[task(binds = LPUART2, shared = [lpuart2])]
    fn lpuart2_interrupt(mut cx: lpuart2_interrupt::Context) {
        cx.shared.lpuart2.lock(|lpuart2| {
            let status = lpuart2.status();
            lpuart2.clear_status(lpuart::Status::RECEIVE_FULL);

            if status.contains(lpuart::Status::RECEIVE_FULL) {
                while !lpuart2.read_data().flags().contains(lpuart::ReadFlags::RXEMPT) {
                    let data = lpuart2.read_data();
                    while !lpuart2.status().contains(lpuart::Status::TRANSMIT_EMPTY) {}
                    lpuart2.write_byte(data.into());
                }
            }
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }
}
