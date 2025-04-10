use core::fmt::Display;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin, PinState},
};

//// From embedded-dht-rs = { version = "0.4.0", features = ["dht11"] }
//// Info pins https://www.upesy.fr/blogs/tutorials/dht11-humidity-temperature-sensor-with-arduino-code-on-esp32-board

/// Possible errors when interacting with the sensor.
#[derive(Debug)]
pub enum SensorError {
    ChecksumMismatch,
    Timeout,
    PinError,
}

const DEFAULT_MAX_ATTEMPTS: usize = 10_000;

/// Common base struct for DHT11, DHT22 sensors.
pub struct Dht<P: InputPin + OutputPin, D: DelayNs> {
    pin: P,
    delay: D,
}

impl<P: InputPin + OutputPin, D: DelayNs> Dht<P, D> {
    pub fn new(pin: P, delay: D) -> Self {
        Self { pin, delay }
    }

    /// Reads a byte (8 bits) from the sensor.
    ///
    /// This method reads 8 bits sequentially from the sensor to construct a byte.
    /// It follows the communication protocol of the DHT11/DHT22 sensors:
    ///
    /// For each bit:
    /// - Waits for the pin to go **high** (start of bit transmission).
    /// - Delays for **30 microseconds** to sample the bit value.
    ///   - If the pin is **high** after the delay, the bit is interpreted as **'1'**.
    ///   - If the pin is **low**, the bit is interpreted as **'0'**.
    /// - Waits for the pin to go **low** (end of bit transmission).
    ///
    /// The bits are assembled into a byte, starting from the most significant bit (MSB).
    ///
    /// # Returns
    ///
    /// - `Ok(u8)`: The byte read from the sensor.
    /// - `Err(SensorError<P::Error>)`: If a pin error occurs.
    pub fn read_byte(&mut self) -> Result<u8, SensorError> {
        let mut byte: u8 = 0;
        for n in 0..8 {
            self.wait_until_state(PinState::High)?;
            self.delay.delay_us(30);
            let is_bit_1 = self.pin.is_high();
            if is_bit_1.unwrap() {
                let bit_mask = 1 << (7 - (n % 8));
                byte |= bit_mask;
                self.wait_until_state(PinState::Low)?;
            }
        }
        Ok(byte)
    }

    /// Waits until the pin reaches the specified state.
    ///
    /// This helper function continuously polls the pin until it reaches the desired `PinState`.
    /// It introduces a **1-microsecond delay** between each poll to prevent excessive CPU usage.
    ///
    /// # Arguments
    ///
    /// - `state`: The target `PinState` to wait for (`PinState::High` or `PinState::Low`).
    ///
    /// # Returns
    ///
    /// - `Ok(())`: When the pin reaches the desired state.
    /// - `Err(SensorError::Timeout)`: If the desired state is not reached in time.
    /// - `Err(SensorError::Io(...))`: If a pin error occurs while reading the pin state.
    ///
    pub fn wait_until_state(&mut self, state: PinState) -> Result<(), SensorError> {
        for _ in 0..DEFAULT_MAX_ATTEMPTS {
            let is_state = match state {
                PinState::Low => self.pin.is_low(),
                PinState::High => self.pin.is_high(),
            };

            match is_state {
                Ok(true) => return Ok(()),
                Ok(false) => self.delay.delay_us(1),
                Err(_) => return Err(SensorError::PinError),
            }
        }

        Err(SensorError::Timeout)
    }
}

pub struct Dht11<P: InputPin + OutputPin, D: DelayNs> {
    dht: Dht<P, D>,
}

impl<P: InputPin + OutputPin, D: DelayNs> Dht11<P, D> {
    pub fn new(pin: P, delay: D) -> Self {
        Self {
            dht: Dht::new(pin, delay),
        }
    }

    pub fn read(&mut self) -> Result<SensorReading<u8>, SensorError> {
        // Start communication: pull pin low for 18ms, then release.
        let _ = self.dht.pin.set_low();
        self.dht.delay.delay_ms(18);
        let _ = self.dht.pin.set_high();

        // Wait for sensor to respond.
        self.dht.delay.delay_us(48);

        // Sync with sensor: wait for high then low signals.
        let _ = self.dht.wait_until_state(PinState::High);
        let _ = self.dht.wait_until_state(PinState::Low);

        // Start reading 40 bits
        let humidity_integer = self.dht.read_byte()?;
        let humidity_decimal = self.dht.read_byte()?;
        let temperature_integer = self.dht.read_byte()?;
        let temperature_decimal = self.dht.read_byte()?;
        let checksum = self.dht.read_byte()?;

        // Checksum
        let sum = humidity_integer
            .wrapping_add(humidity_decimal)
            .wrapping_add(temperature_integer)
            .wrapping_add(temperature_decimal);
        if sum != checksum {
            return Err(SensorError::ChecksumMismatch);
        }

        Ok(SensorReading {
            humidity: humidity_integer,
            temperature: temperature_integer,
        })
    }
}

/// Represents a reading from the sensor.
pub struct SensorReading<T> {
    humidity: T,
    temperature: T,
}

impl<T: Clone + Copy> SensorReading<T> {
    pub fn humidity(&self) -> T {
        self.humidity
    }

    pub fn temperature(&self) -> T {
        self.temperature
    }
}

impl<T: Display> Display for SensorReading<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "Temperature: {} °C, Humidity: {} %",
            self.temperature, self.humidity
        )
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use embedded_hal_mock::eh1::digital::{Mock, State, Transaction as PinTransaction};
//     use embedded_hal_mock::eh1::delay::NoopDelay as MockNoop;

//     #[test]
//     fn test_read_byte() {
//     // Set up the pin transactions to mock the behavior of the sensor during the reading of a byte.
//     // Each bit read from the sensor starts with a High state that lasts long enough
//     // to signify the bit, followed by reading whether it stays High (bit 1) or goes Low (bit 0).
//     let expectations = [
//         // Bit 1 - 0
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 2 - 1
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 3 - 0
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 4 - 1
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 5 - 0
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 6 - 1
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 7 - 1
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//         // Bit 8 - 1
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::High),
//         PinTransaction::get(State::Low),

//     ];

//         let mock_pin = Mock::new(&expectations);
//         let mock_delay = MockNoop::new();

//         let mut dht = Dht::new(mock_pin, mock_delay);

//         let result = dht.read_byte().unwrap();
//         assert_eq!(result, 0b01010111);

//         dht.pin.done();
//     }

//     #[test]
//     fn test_wait_until_state() {
//         let expectations = [
//             PinTransaction::get(State::Low),
//             PinTransaction::get(State::Low),
//             PinTransaction::get(State::High),
//         ];

//         let mock_pin = Mock::new(&expectations);
//         let mock_delay = MockNoop::new();

//         let mut dht = Dht::new(mock_pin, mock_delay);

//         let result = dht.wait_until_state(PinState::High);
//         assert!(result.is_ok());

//         dht.pin.done();
//     }
// }
