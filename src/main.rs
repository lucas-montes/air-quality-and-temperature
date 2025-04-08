#![no_std]
#![no_main]

mod dht11;
mod ens160_plus_aht21;
mod mq135;
mod utils;

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
    i2c::master::{BusTimeout, Config, I2c},
    main,
    peripherals::ADC1,
};

use dht11::Dht11;
use ens160_plus_aht21::Aht21;
use mq135::MQ135;
use ssd1306::{
    prelude::{DisplayRotation, *},
    size::DisplaySize128x64,
    I2CDisplayInterface, Ssd1306,
};
use utils::{RollingMedian, StrWriter};

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("error {}", info);
    esp_hal::system::software_reset()
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

    let mut i2c = match I2c::new(
        peripherals.I2C0,
        Config::default().with_timeout(BusTimeout::Maximum),
    ) {
        Ok(i2c) => i2c
            .with_sda(peripherals.GPIO21)
            .with_scl(peripherals.GPIO22),
        Err(e) => {
            log::error!("I2C error: {:?}", e);
            loop {}
        }
    };
    let i2c_cell = AtomicCell::new(i2c);

    let mut aht = Aht21::new(AtomicDevice::new(&i2c_cell), delay);
    let mut ens160 = Ens160::new(AtomicDevice::new(&i2c_cell), 0x53);

    let mut ready = false;

    let mut ready1 = false;
    loop {
        if ready && ready1 {
            break;
        }
        delay.delay_ms(3000);
        if !ready {
            if let Err(err) = aht.init() {
                log::error!("AHT error: {:?}", err);
            } else {
                ready = true;
            }
        }

        if !ready1 {
            delay.delay_ms(500);
            if let Err(err) = ens160.operational() {
                log::error!("ens160 error: {:?}", err);
            } else {
                ready1 = true;
            }
            delay.delay_ms(500);
        }
    }

    let interface = I2CDisplayInterface::new(AtomicDevice::new(&i2c_cell));

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    log::info!("all started well");
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let mut temperature_average = RollingMedian::<512, f32>::new("Temperature"); // around 2 Kib
    let mut humidity_average = RollingMedian::<512, f32>::new("Humidity"); // around 2 Kib
    let mut eco2_average = RollingMedian::<512, u16>::new("eCO2"); // around 2 Kib
    let mut tvoc_average = RollingMedian::<512, u16>::new("TVOC ppb"); // around 2 Kib
    let mut c02_average = RollingMedian::<512, f32>::new("CO2 ppm"); // around 2 Kib

    loop {
        delay.delay_ms(500);

        display.clear_buffer();

        // Get air quality from Ens160
        let quality_str = match ens160.air_quality_index().unwrap() {
            AirQualityIndex::Excellent => "Air Quality: Excellent",
            AirQualityIndex::Good => "Air Quality: Good",
            AirQualityIndex::Moderate => "Air Quality: Moderate",
            AirQualityIndex::Poor => "Air Quality: Poor",
            AirQualityIndex::Unhealthy => "Air Quality: Unhealthy",
        };
        Text::with_baseline(quality_str, Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        if let Ok(co2_ppm) = senseur.get_ppm() {
            c02_average.update(co2_ppm);
        };
        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", &c02_average);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 12),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        // Get Eco2 from Ens160
        if let Ok(eco2) = ens160.eco2() {
            eco2_average.update(*eco2);
        }

        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", &eco2_average);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 22),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        // Get TVOC from Ens160
        if let Ok(tvoc) = ens160.tvoc() {
            tvoc_average.update(tvoc);
        }
        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", &tvoc_average);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 32),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        // Read ENS160
        if let Ok((raw_temp, raw_hum)) = ens160.temp_and_hum() {
            let temp = raw_temp as f32 / 100.0;
            let hum = raw_hum as f32 / 100.0;
            temperature_average.update(temp);
            humidity_average.update(hum);
        }

        // Read DHT11
        if let Ok(reading) = dht11.read() {
            temperature_average.update(reading.temperature() as f32);
            humidity_average.update(reading.humidity() as f32);
        }

        // Read AHT21
        if let Ok((hum, temp)) = aht.read() {
            temperature_average.update(temp.celsius());
            humidity_average.update(hum.rh());
        }

        let mut writer = StrWriter {
            buffer: &mut [0u8; 32],
            pos: 0,
        };
        let _ = write!(writer, "{}", &temperature_average);
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
        let _ = write!(writer, "{}", &humidity_average);
        Text::with_baseline(
            writer.as_str(),
            Point::new(0, 52),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        display.flush().unwrap();
    }
}
