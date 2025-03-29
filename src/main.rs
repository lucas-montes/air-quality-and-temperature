#![no_std]
#![no_main]

use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    main,
    time::{Duration, Instant},
};

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
    let button = Input::new(peripherals.GPIO5, InputConfig::default());
    let mut led = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    loop {
        if button.is_low() {
            self_led.toggle();
            led.set_high();
        } else {
            self_led.set_high();
            led.toggle();
        };
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(250) {}
    }
}
