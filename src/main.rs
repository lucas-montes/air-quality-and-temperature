#![no_std]
#![no_main]

mod dht11;
mod mq135;

use esp_hal::{
    analog::adc::{ Adc, AdcConfig, Attenuation}, clock::CpuClock, delay::Delay, gpio::{ Flex,  Level, Output, OutputConfig, Pull}, main, peripherals::ADC1, time::{Duration, Instant}
};


use dht11::Dht11;
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

    let mut self_led = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());

    let delay = Delay::new();
    let mut capteur = Flex::new(peripherals.GPIO5);
    capteur.set_as_open_drain(Pull::None);
    let mut dht11 = Dht11::new(capteur, delay.clone());


    let mut adc_config: AdcConfig<ADC1> = AdcConfig::new();
    let adc_pin =  adc_config.enable_pin(peripherals.GPIO34, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);
    let mut senseur = MQ135::new(adc_pin, &mut adc1, delay);

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
        self_led.toggle();

        if let Ok(value) = senseur.get_ppm() {
            log::info!("CO2 Value: {}", value);
        }

        match dht11.read() {
            Ok(sensor_reading) => log::info!(
                "DHT 11 Sensor - {}",
                sensor_reading
            ),
            Err(error) => log::error!("An error occurred while trying to read sensor: {:?}", error),
        }

        log::info!("-----");
    }
}
