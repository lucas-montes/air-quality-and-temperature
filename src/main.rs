#![no_std]
#![no_main]

mod dht11;
mod ens160_plus_aht21;
mod mq135;

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
    Drawable,
};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use ens160::{AirQualityIndex, Ens160};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::CpuClock,
    delay::Delay,
    gpio::{Flex, Pull},
    i2c::master::{Config, I2c},
    main,
    peripherals::ADC1,
    time::{Duration, Instant},
};

use dht11::Dht11;
use ens160_plus_aht21::Aht21;
use mq135::MQ135;
use ssd1306::{
    prelude::{DisplayRotation, *},
    size::DisplaySize128x64,
    I2CDisplayInterface, Ssd1306,
};

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::info!("error {}", info);
    esp_hal::system::software_reset()
}

struct StrWriter<'a> {
    buffer: &'a mut [u8],
    pos: usize,
}

impl<'a> core::fmt::Write for StrWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let len = bytes.len();

        if self.pos + len > self.buffer.len() {
            return Err(core::fmt::Error);
        }

        self.buffer[self.pos..self.pos + len].copy_from_slice(bytes);
        self.pos += len;
        Ok(())
    }
}

impl<'a> StrWriter<'a> {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buffer[..self.pos]).unwrap_or("")
    }
}

struct PrintableResult<'a, T> {
    value: T,
    title: &'a str,
}

impl<'a, T: core::fmt::Display> core::fmt::Display for PrintableResult<'a, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{}: {}", self.title, self.value)
    }
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut delay = Delay::new();
    let mut capteur = Flex::new(peripherals.GPIO5);
    capteur.set_as_open_drain(Pull::None);
    let mut dht11 = Dht11::new(capteur, delay);

    let mut adc_config: AdcConfig<ADC1> = AdcConfig::new();
    let adc_pin = adc_config.enable_pin(peripherals.GPIO34, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);
    let mut senseur = MQ135::new(adc_pin, &mut adc1, delay);

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .expect("error with i2c")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    let i2c_cell = AtomicCell::new(i2c);

    let mut aht = Aht21::new(AtomicDevice::new(&i2c_cell), delay).expect("aht error");

    let mut ens160 = Ens160::new(AtomicDevice::new(&i2c_cell), 0x53);
    delay.delay_ms(500);
    ens160.operational().unwrap();
    delay.delay_ms(500);

    let interface = I2CDisplayInterface::new(AtomicDevice::new(&i2c_cell));

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}

        display.clear_buffer();
        // Get CO2 values

        // Get air quality from Ens160
        let quality = ens160.air_quality_index().unwrap();

        let quality_str = match quality {
            AirQualityIndex::Excellent => "Air Quality: Excellent",
            AirQualityIndex::Good => "Air Quality: Good",
            AirQualityIndex::Moderate => "Air Quality: Moderate",
            AirQualityIndex::Poor => "Air Quality: Poor",
            AirQualityIndex::Unhealthy => "Air Quality: Unhealthy",
        };
        Text::with_baseline(quality_str, Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        let co2_ppm = senseur.get_ppm().unwrap_or(400.0);
        let result = PrintableResult {
            value: co2_ppm,
            title: "CO2 ppm",
        };
        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", result);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 12),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        // Get Eco2 from Ens160
        let eco2 = ens160.eco2().unwrap();
        let result = PrintableResult {
            value: *eco2,
            title: "eCO2",
        };

        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", result);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 22),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        // Get TVOC from Ens160
        let tvoc = ens160.tvoc().unwrap();
        let result = PrintableResult {
            value: tvoc,
            title: "TVOC ppb",
        };
        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", result);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 32),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        let mut total_temp: f32 = 0.0;
        let mut total_hum: f32 = 0.0;
        let mut count = 0.0;

        // Read ENS160
        if let Ok((raw_temp, raw_hum)) = ens160.temp_and_hum() {
            let temp = raw_temp as f32 / 100.0;
            let hum = raw_hum as f32 / 100.0;
            log::info!("ENS160: {}°C, {}%", temp, hum);
            total_temp += temp;
            total_hum += hum;
            count += 1.0;
        }

        // Read DHT11
        if let Ok(reading) = dht11.read() {
            total_temp += reading.temperature() as f32;
            total_hum += reading.humidity() as f32;
            log::info!(
                "DHT11: {}°C, {}%",
                reading.temperature() as f32,
                reading.humidity() as f32
            );
            count += 1.0;
        } else {
            log::warn!("Failed to read from DHT11");
        }

        // Read AHT21
        if let Ok((hum, temp)) = aht.read() {
            total_temp += temp.celsius();
            total_hum += hum.rh();
            log::info!("AHT21: {}°C, {}%", temp.celsius(), hum.rh());
            count += 1.0;
        } else {
            log::warn!("Failed to read from AHT21");
        }

        // Calculate averages if any valid reading was made
        if count > 0.0 {
            let avg_temp = total_temp / count;
            let avg_hum = total_hum / count;

            let result_avg_temp = PrintableResult {
                value: avg_temp,
                title: "Temperature",
            };

            let result_avg_hum = PrintableResult {
                value: avg_hum,
                title: "Humidity",
            };

            let mut writer = StrWriter {
                buffer: &mut [0u8; 32],
                pos: 0,
            };
            let _ = write!(writer, "{}", result_avg_temp);
            Text::with_baseline(
                writer.as_str(),
                Point::new(0, 42),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .unwrap();

            let mut writer = StrWriter {
                buffer: &mut [0u8; 32],
                pos: 0,
            };
            let _ = write!(writer, "{}", result_avg_hum);
            Text::with_baseline(
                writer.as_str(),
                Point::new(0, 52),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .unwrap();
        }

        display.flush().unwrap();
    }
}
