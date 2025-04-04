#![no_std]
#![no_main]

mod dht11;
mod ens160_plus_aht21;
mod mq135;

use embedded_hal::delay::DelayNs;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use ens160::Ens160;
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

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::info!("error {}", info);
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

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}

        if let Ok(value) = senseur.get_ppm() {
            log::info!("CO2 Value: {}", value);
        }

        let quality = ens160.air_quality_index().unwrap();
        let temp_hum = ens160.temp_and_hum().unwrap();
        let eco2 = ens160.eco2().unwrap();
        let tvoc = ens160.tvoc().unwrap();

        log::info!("quality: {:?}", quality);
        log::info!("eco2: {:?}", eco2);
        log::info!("tvoc: {:?}", tvoc);
        log::info!("temp hum: {:?}", temp_hum); // TODO: is in integer needs to be in float

        match dht11.read() {
            Ok(sensor_reading) => log::info!("DHT 11 Sensor - {}", sensor_reading),
            Err(error) => log::error!("An error occurred while trying to read sensor: {:?}", error),
        }

        match aht.read() {
            Ok((h, t)) => log::info!(
                "relative humidity={0}%; temperature={1}C",
                h.rh(),
                t.celsius()
            ),

            Err(error) => log::error!("An error occurred while trying to read sensor: {:?}", error),
        }

        log::info!("-----");
    }
}
