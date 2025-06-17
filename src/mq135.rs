use embedded_hal::delay::DelayNs;
use micromath::F32Ext;

use esp_hal::{
    analog::adc::{AdcPin, Adc},
    gpio::GpioPin,
    peripherals::ADC1,
};
///TODO: use different traits or something to calculate all gases


/// Parameters for calculating ppm of CO2 from sensor resistance
const PARA: f32 = 116.6020682;
const PARB: f32 = 2.769034857;

/// Parameters for calculating ppm of benzen from sensor resistance
// const PARA: f32 = 23.4461175;
// const PARB: f32 = -3.98724605;


/// Parameters to model temperature and humidity dependence
const CORA: f32 = 0.00035;
const CORB: f32 = 0.02718;
const CORC: f32 = 1.39538;
const CORD: f32 = 0.0018;

/// Atmospheric CO2 level for calibration purposes
/// Get current value from https://www.co2.earth/
const ATMOCO2: f32 = 427.09;

const RLOAD: f32 = 10.0; // The load resistance on the board in kOhms
const RZERO: f32 = 76.63; // Calibration resistance at atmospheric CO2 level

	//for arduino uno/nano ADC is 10bit with 5v reference (default)
	//for bluepill on stm32 ADC is 12 bit with 3.3v reference (can be set in constructor)
const ADC_MAX: u32 = 4095; // ESP32 12-bit ADC
const VREF: f32 = 4.9; // Reference voltage (5.0V)


#[derive(Debug)]
pub enum SensorError {
    PinError
}

// pub struct SoilMoisture<'a, ADCI: RegisterAccess + 'a, PIN: AdcChannel + AnalogPin, D: DelayNs> {
//     pin: AdcPin<PIN, ADCI>,
//     adc: Adc<'a, ADCI, esp_hal::Blocking>,
//     delay: D,
// }

// impl<'a, ADCI: RegisterAccess + 'a,PIN: AdcChannel + AnalogPin,D: DelayNs> SoilMoisture<'a,ADCI,PIN, D> {
//     /// Create a new MQ135 instance
//     pub fn new(adc_instance: impl Peripheral<P = ADCI> + 'a,pin: PIN, attenuation: Attenuation,delay: D) -> Self {
//         let mut adc_config: AdcConfig<ADCI> = AdcConfig::new();
//     let adc_pin = adc_config.enable_pin(pin, attenuation);
//     let adc = Adc::new(adc_instance, adc_config);

//         Self {
//             pin: adc_pin,
//             adc,
//             delay,
//         }
//     }

pub struct MQ135<'a, D: DelayNs> {
    pin: AdcPin<GpioPin<34>, ADC1>, // GPIO35 for AO
    adc: &'a mut Adc<'a, ADC1, esp_hal::Blocking>,
    delay: D,
}

impl<'a, D: DelayNs> MQ135<'a, D> {
    /// Create a new MQ135 instance
    pub fn new(pin: AdcPin<GpioPin<34>, ADC1>, adc: &'a mut Adc<'a, ADC1, esp_hal::Blocking>,delay: D) -> Self {
        MQ135 { pin, adc, delay }
    }

    /// Read the raw ADC value (equivalent to analogRead)
    fn read_adc(&mut self) -> Result<u16, SensorError> {
        self.delay.delay_ms(250);
        self.adc.read_oneshot(&mut self.pin).map_err(|err|SensorError::PinError)

    }

    /// Get the correction factor to correct for temperature and humidity
    pub fn get_correction_factor(&self, temperature: f32, humidity: f32) -> Result<f32, SensorError> {
        Ok(CORA * temperature * temperature - CORB * temperature + CORC - (humidity - 33.0) * CORD)
    }

    /// Get the resistance of the sensor, ie. the measurement value
    pub fn get_resistance(&mut self) -> Result<f32, SensorError> {
        let val = self.read_adc()? as f32;
        if val == 0.0 {
            return Ok(f32::INFINITY); // Avoid division by zero
        }
        Ok(((ADC_MAX as f32 / val) * VREF - 1.0) * RLOAD)
    }

    /// Get the resistance of the sensor, ie. the measurement value corrected for temp/hum
    /// return The corrected sensor resistance kOhm
    pub fn get_corrected_resistance(&mut self, temperature: f32, humidity: f32) -> Result<f32, SensorError> {
        Ok(self.get_resistance()? / self.get_correction_factor(temperature, humidity)?)
    }

    /// Get the PPM of CO2 (assuming only CO2 in the air)
    pub fn get_ppm(&mut self) -> Result<f32, SensorError> {
        Ok((self.get_resistance()? / RZERO).powf(-PARB) * PARA)
    }

    /// Get the corrected PPM of CO2 in the air
    pub fn get_corrected_ppm(&mut self,temperature: f32, humidity: f32) -> Result<f32, SensorError> {
       Ok( PARA * f32::powf(self.get_corrected_resistance(temperature, humidity)? / RZERO, -PARB))
    }

    /// Get the resistance RZero of the sensor for calibration purposes
    /// return The sensor resistance RZero in kOhm
    pub fn get_rzero(&mut self) -> Result<f32, SensorError> {
        Ok(self.get_resistance()? * f32::powf(ATMOCO2 / PARA, 1.0 / PARB))
    }

    /// Get the corrected resistance RZero of the sensor for calibration purposes
    /// return The corrected sensor resistance RZero in kOhm
    pub fn get_corrected_rzero(&mut self,temperature: f32, humidity: f32) -> Result<f32, SensorError> {
        Ok(self.get_corrected_resistance(temperature, humidity)? * f32::powf(ATMOCO2 / PARA, 1.0 / PARB))
    }
}
