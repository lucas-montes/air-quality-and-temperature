#![no_std]
#![no_main]

use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
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
    log::info!("Hello world!");
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    log::info!("Hello world!");
    let peripherals = esp_hal::init(config);
    log::info!("Hello world!");

    // Set GPIO0 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO0, Level::High, OutputConfig::default());
    log::info!("Hello world!");
    loop {
        led.toggle();
        // Wait for half a second
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }
}
