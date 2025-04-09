#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

mod dht11;
mod ens160_plus_aht21;
mod mq135;
mod utils;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use core::{fmt::Write, ptr::addr_of_mut};
use embassy_executor::Spawner;
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
    peripherals::ADC1,
    rng::Rng,
    system::{Cpu, CpuControl, Stack},
    time,
    timer::timg::TimerGroup,
};
use esp_hal_embassy::Executor;
use esp_println::logger;
use static_cell::StaticCell;

use dht11::Dht11;
use ens160_plus_aht21::Aht21;
use esp_wifi::{ble::controller::BleConnector, init, EspWifiController};
use mq135::MQ135;
use ssd1306::{
    prelude::{DisplayRotation, *},
    size::DisplaySize128x64,
    I2CDisplayInterface, Ssd1306,
};
use utils::{RollingMedian, StrWriter};

extern crate alloc;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

// You need a panic handler. Usually, you you would use esp_backtrace, panic-probe, or
// something similar, but you can also bring your own like this:
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("error {}", info);
    loop {}
}

#[alloc_error_handler]
fn alloc_error(layout: core::alloc::Layout) -> ! {
    log::error!("Memory allocation error: {:?}", layout);
    loop {}
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_alloc::heap_allocator!(size: 72 * 1024);
    logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();
    let mut capteur = Flex::new(peripherals.GPIO5);
    capteur.set_as_open_drain(Pull::None);
    let dht11 = Dht11::new(capteur, delay);

    let mut adc_config: AdcConfig<ADC1> = AdcConfig::new();
    let adc_pin = adc_config.enable_pin(peripherals.GPIO34, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);
    let senseur = MQ135::new(adc_pin, &mut adc1, delay);

    let i2c = match I2c::new(
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

    let aht = Aht21::new(AtomicDevice::new(&i2c_cell), delay);
    let ens160 = Ens160::new(AtomicDevice::new(&i2c_cell), 0x53);

    let interface = I2CDisplayInterface::new(AtomicDevice::new(&i2c_cell));

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    log::info!("all started well");

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);
    let bluetooth = peripherals.BT;
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    let _guard =
        match cpu_control.start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                if let Err(err) = spawner.spawn(bluetooth_loop(esp_wifi_ctrl, bluetooth)) {
                    log::error!("spawnner {}", err);
                }
            });
        }) {
            Ok(g) => g,
            Err(err) => {
                log::error!("Error starting app core: {:?}", err);
                loop {}
            }
        };
    data_loop(dht11, senseur, aht, ens160, delay, display).await
}

#[embassy_executor::task]
async fn bluetooth_loop(
    esp_wifi_ctrl: &'static EspWifiController<'static>,
    mut bluetooth: esp_hal::peripherals::BT,
) {
    log::info!("bluetooth started {}", Cpu::current() as usize);
    let connector = BleConnector::new(esp_wifi_ctrl, &mut bluetooth);

    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let mut ble = Ble::new(connector, now);
    log::info!("ble inint {:?}", ble.init().await);
    log::info!("cmd {:?}", ble.cmd_set_le_advertising_parameters().await);
    log::info!(
        "{:?}",
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::CompleteLocalName("RoomTempIq"),
                // AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
            ])
            .unwrap()
        )
        .await
    );
    log::info!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

    log::info!("started advertising");

    let sensor_data = b"Value: 80";

    let mut read_func = |_offset: usize, data: &mut [u8]| {
        data[0..sensor_data.len()].copy_from_slice(&sensor_data[..]);
        sensor_data.len()
    };

    gatt!([service {
        uuid: "a9c81b72-0f7a-4c59-b0a8-425e3bcf0a0e",
        characteristics: [characteristic {
            name: "my_characteristic",
            uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
            notify: true,
            read: read_func,
        }],
    },]);

    let mut no_rng = bleps::no_rng::NoRng;
    let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut no_rng);

    let mut notifier =
        || async { NotificationData::new(my_characteristic_handle, "Temp:24".as_bytes()) };

    if let Err(err) = srv.run(&mut notifier).await {
        log::error!("Error bluetooth: {:?}", err);
    };
}

async fn data_loop<'a>(
    mut dht11: Dht11<Flex<'a>, Delay>,
    mut senseur: MQ135<'a, Delay>,
    mut aht: Aht21<AtomicDevice<'a, I2c<'a, esp_hal::Blocking>>, Delay>,
    mut ens160: Ens160<AtomicDevice<'a, I2c<'a, esp_hal::Blocking>>>,
    mut delay: Delay,
    mut display: Ssd1306<
        I2CInterface<AtomicDevice<'a, I2c<'a, esp_hal::Blocking>>>,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
    >,
) {
    log::info!("data started {}", Cpu::current() as usize);
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

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let mut temperature_average = RollingMedian::<256, f32>::new("Temperature");
    let mut humidity_average = RollingMedian::<256, f32>::new("Humidity");
    let mut eco2_average = RollingMedian::<256, u16>::new("eCO2");
    let mut tvoc_average = RollingMedian::<256, u16>::new("TVOC ppb");
    let mut c02_average = RollingMedian::<256, f32>::new("CO2 ppm");
    let mut show_heap = 0;
    loop {
        show_heap += 1;
        delay.delay_ms(500);

        display.clear_buffer();

        if show_heap % 3 == 0 {
            log::info!("{}", esp_alloc::HEAP.stats());
        };
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
            log::info!("CO2 PPM: {}", co2_ppm);
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
            log::info!("eCO2: {}", *eco2);
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
            log::info!("TVOC: {}", tvoc);
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
            log::info!("ENS160 temp: {} C, hum: {} %", temp, hum);
            temperature_average.update(temp);
            humidity_average.update(hum);
        }

        // Read DHT11
        if let Ok(reading) = dht11.read() {
            log::info!(
                "DHT11 temp: {} C, hum: {} %",
                reading.temperature(),
                reading.humidity()
            );
            temperature_average.update(reading.temperature() as f32);
            humidity_average.update(reading.humidity() as f32);
        }

        // Read AHT21
        if let Ok((hum, temp)) = aht.read() {
            log::info!("AHT21 temp: {} C, hum: {} %", temp.celsius(), hum.rh());
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
