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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
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
    gpio::{Flex, Level, Output, OutputConfig, Pull},
    i2c::master::{BusTimeout, Config, I2c},
    peripherals::ADC1,
    rng::Rng,
    system::{Cpu, CpuControl, Stack},
    time,
    timer::{timg::TimerGroup, AnyTimer},
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

use embassy_time::Timer;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("big error happened {}", info);
    esp_hal::system::software_reset();
}

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

/// Waits for a message that contains a duration, then flashes a led for that
/// duration of time.
#[embassy_executor::task]
async fn control_led(
    mut led: Output<'static>,
    control: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    log::info!("Starting control_led() on core {}", Cpu::current() as usize);
    loop {
        if control.wait().await {
            log::info!("LED on");
            led.set_low();
        } else {
            log::info!("LED off");
            led.set_high();
        }
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[link_section = ".dram2_uninit"] size: 72 * 1024);

    let timg = TimerGroup::new(peripherals.TIMG0);
    let rando = Rng::new(peripherals.RNG);
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg.timer0, rando, peripherals.RADIO_CLK,).unwrap()
    );

    let timg0 = TimerGroup::new(peripherals.TIMG1);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();
    esp_hal_embassy::init([timer0, timer1]);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    let led_ctrl_signal = &*LED_CTRL.init(Signal::new());


    log::info!("all started well");

    let _guard =
        match cpu_control.start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                if let Err(err) = spawner.spawn(data_loop(
                    peripherals.GPIO5,
                    peripherals.GPIO21,
                    peripherals.GPIO22,
                    peripherals.GPIO34,
                    peripherals.ADC1,
                    peripherals.I2C0,
                )) {
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

    let mut bluetooth = peripherals.BT;

    let connector = BleConnector::new(&esp_wifi_ctrl, &mut bluetooth);

    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let mut ble = Ble::new(connector, now);
    log::info!("Connector created");

    loop {
        log::info!("{:?}", ble.init().await);
        log::info!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        log::info!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName(esp_hal::chip!()),
                ])
                .unwrap()
            )
            .await
        );
        log::info!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

        log::info!("started advertising");

        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            17
        };
        let mut wf = |offset: usize, data: &[u8]| {
            log::info!("RECEIVED: {} {:?}", offset, data);
            log::info!("Sending LED on");
            led_ctrl_signal.signal(true);
        };

        let mut wf2 = |offset: usize, data: &[u8]| {
            log::info!("RECEIVED: {} {:?}", offset, data);
            log::info!("Sending LED off");
            led_ctrl_signal.signal(false);
        };

        let mut rf3 = |_offset: usize, data: &mut [u8]| {
            data[..5].copy_from_slice(&b"Hola!"[..]);
            5
        };
        let mut wf3 = |offset: usize, data: &[u8]| {
            log::info!("RECEIVED: Offset {}, data {:?}", offset, data);
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [
                characteristic {
                    name: "my_characteristic",
                    uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
                    notify: true,
                    read: rf3,
                    write: wf3,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let counter = core::cell::RefCell::new(0u8);
        let counter = &counter;

        let mut notifier = || {
            // TODO how to check if notifications are enabled for the characteristic?
            // maybe pass something into the closure which just can query the characteristic
            // value probably passing in the attribute server won't work?

            async {
                let mut val;

                loop {
                    val = scale_random_u32(rando.clone().random());
                    if val >= 100 {
                        break;
                    }
                    Timer::after_millis(500).await;
                }
                let mut data = [0u8; 13];
                data.copy_from_slice(b"Notification0");
                {
                    let mut counter = counter.borrow_mut();
                    data[data.len() - 1] += *counter;
                    *counter = (*counter + 1) % 10;
                }
                NotificationData::new(my_characteristic_handle, &data)
            }
        };

        srv.run(&mut notifier).await.unwrap();
    }
}

const fn scale_random_u32(rand_val: u32) -> u32 {
    let min = 60;
    let max = 110;
    min + (rand_val % (max - min + 1))
}


#[embassy_executor::task]
async fn data_loop(
    dht11_pin: esp_hal::gpio::GpioPin<5>,
    sda_pin: esp_hal::gpio::GpioPin<21>,
    scl_pin: esp_hal::gpio::GpioPin<22>,
    adc_pin: esp_hal::gpio::GpioPin<34>,
    adc1: ADC1,
    i2c0: esp_hal::peripherals::I2C0,
) {
    log::info!("data started {}", Cpu::current() as usize);
    let mut delay = Delay::new();
    let mut capteur = Flex::new(dht11_pin);
    capteur.set_as_open_drain(Pull::None);
    let mut dht11 = Dht11::new(capteur, delay);

    let mut adc_config: AdcConfig<ADC1> = AdcConfig::new();
    let enable_adc_pin = adc_config.enable_pin(adc_pin, Attenuation::_11dB);
    let mut adc1 = Adc::new(adc1, adc_config);
    let mut senseur = MQ135::new(enable_adc_pin, &mut adc1, delay);

    let i2c = match I2c::new(i2c0, Config::default().with_timeout(BusTimeout::Maximum)) {
        Ok(i2c) => i2c.with_sda(sda_pin).with_scl(scl_pin),
        Err(e) => {
            log::error!("I2C error: {:?}", e);
            loop {}
        }
    };
    let i2c_cell = AtomicCell::new(i2c);

    let mut aht = Aht21::new(AtomicDevice::new(&i2c_cell), delay);
    let mut ens160 = Ens160::new(AtomicDevice::new(&i2c_cell), 0x53);

    let interface = I2CDisplayInterface::new(AtomicDevice::new(&i2c_cell));

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    log::info!("all started well");

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
            log::info!("memory core 1");
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
            //log::info!("CO2 PPM: {}", co2_ppm);
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
            // log::info!("eCO2: {}", *eco2);
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
            // log::info!("TVOC: {}", tvoc);
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
            // log::info!(
            //     "DHT11 temp: {} C, hum: {} %",
            //     reading.temperature(),
            //     reading.humidity()
            // );
            temperature_average.update(reading.temperature() as f32);
            humidity_average.update(reading.humidity() as f32);
        }

        // Read AHT21
        if let Ok((hum, temp)) = aht.read() {
            // log::info!("AHT21 temp: {} C, hum: {} %", temp.celsius(), hum.rh());
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
