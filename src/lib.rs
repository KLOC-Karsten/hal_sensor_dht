//! An interface to the DHT Digital Humidity and Temperature sensors.
//!
//! # Design
//! The design of this crate is inspired by:
//! - The [Adatfruit DHT cpp](https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp) code
//! - The [dht-hal-drv](https://crates.io/crates/dht-hal-drv) crate
//! - The [dht-hal](https://crates.io/crates/dht-hal) crate
//! - The [dht-sensor](https://crates.io/crates/dht-sensor) crate
//! - The [dht22_pi](https://crates.io/crates/dht22_pi) crate
//!
//! All of these libraries and also this library are basically the same,
//! only the interfaces are a little different.
//!
//! The code has been tested with DHT22 sensor, a Raspberry Pi 3B, and the rppal.
//!
//! 
//! # Issues
//! This library makes use of the traits provided by
//! the [embedded-hal](https://crates.io/crates/embedded-hal) crate.
//! Unfortunately, a lot of stuff is still missing in embedded-hal, such as
//! traits for reconfigurable GPIOs and disabling interrupts.
//! Therefore it should be expected that this library will be change over time.
//!
//! # Usage
//! To create a driver for the DHT sensor, the caller have to provide the
//! following items:
//! - a GPIO pin, which implements the `InputPin` and `OutputPin` traits of `embedded-hal`.
//!   Additionally, it should be able to reconfigure the pin (from output to input).
//!   Unfortunately, this is currently not part of `embedded-hal`, so the trait from
//!   this crate needs to be implemented.
//! - a timer providing the `DelayMs` and `DelayUs` traits.
//! - interrupts need to be suppressed while we are reading the signal line.
//!   This crate provides a simple trait `InterruptCtrl` for this purpose.
//!
//! # Example: Using the library on a Raspberry Pi
//!
//! ## Implementing the GPIO interfaces.
//! You can use `rppal` to control the GPIO pin.
//! However, a wrapper needs to be implemented because of the "orphan" rule for
//! implementation of external traits for external structs.
//!
//! ```
//! extern crate rppal;
//! use rppal::gpio::{Gpio, Mode, PullUpDown};
//! extern crate hal_sensor_dht;
//! use hal_sensor_dht::{DHTSensor, SensorType};
//!
//! struct MyPin(rppal::gpio::IoPin);
//!
//! impl MyPin {
//!     pub fn new(pin: rppal::gpio::Pin) -> MyPin {
//!         MyPin(pin.into_io(Mode::Input))
//!     }
//! }
//!
//! impl InputPin for MyPin {
//!     type Error = <rppal::gpio::IoPin as InputPin>::Error;
//!     fn is_high(&self) -> Result<bool, <rppal::gpio::IoPin as InputPin>::Error> {
//!         Ok(self.0.is_high())
//!     }
//!     fn is_low(&self) -> Result<bool, <rppal::gpio::IoPin as InputPin>::Error> {
//!         Ok(self.0.is_low())
//!     }
//! }
//!
//! impl OutputPin for MyPin {
//!     type Error = <rppal::gpio::IoPin as OutputPin>::Error;
//!     fn set_high(&mut self) -> Result<(), <rppal::gpio::IoPin as OutputPin>::Error> {
//!         Ok(self.0.set_high())
//!     }
//!     fn set_low(&mut self) -> Result<(), <rppal::gpio::IoPin as OutputPin>::Error> {
//!         Ok(self.0.set_low())
//!     }
//! }
//!
//! impl hal_sensor_dht::IoPin for MyPin {
//!     fn set_input_pullup_mode(&mut self) {
//!         self.0.set_mode(Mode::Input);
//!         self.0.set_pullupdown(PullUpDown::PullUp);
//!     }
//!     fn set_output_mode(&mut self) {
//!         self.0.set_mode(Mode::Output);
//!     }
//! }
//! ```
//!
//! ## Implementing the Delay interfaces.
//! The `DelayMs` is no problem, but microsecond delay is. However, only need a
//! a tiny delay, therefore we use write and read operation to produce such small
//! delay.
//!
//! ```
//! use std::thread;
//! use std::time::Duration;
//! use rppal::gpio::{Gpio, Mode, PullUpDown};
//!
//! use std::ptr::read_volatile;
//! use std::ptr::write_volatile;
//! struct MyTimer {}
//!
//! impl DelayUs<u16> for MyTimer {
//!     fn delay_us(&mut self, t:u16) {
//!         let mut i = 0;
//!         unsafe {
//!             while read_volatile(&mut i) < t {
//!                 write_volatile(&mut i, read_volatile(&mut i) + 1);
//!             }
//!         }
//!     }
//! }
//!
//! impl DelayMs<u16> for MyTimer {
//!     fn delay_ms(&mut self, ms: u16) {
//!         thread::sleep(Duration::from_millis(ms.into()));
//!     }
//! }
//! ```
//!
//! ## Disabling Interrupts.
//!
//! You will use `sched_setscheduler` from the `libc` crate for this purpose.
//! This is good enough for reading the sensor data.
//!
//! ```
//! extern crate libc;
//! use libc::sched_param;
//! use libc::sched_setscheduler;
//! use libc::SCHED_FIFO;
//! use libc::SCHED_OTHER;
//!
//! struct MyInterruptCtrl {}
//!
//! impl hal_sensor_dht::InterruptCtrl for MyInterruptCtrl {
//!     fn enable(&mut self) {
//!         unsafe {
//!             let param = sched_param { sched_priority: 32 };
//!             let result = sched_setscheduler(0, SCHED_FIFO, &param);
//!
//!             if result != 0 {
//!                 panic!("Error setting priority, you may not have cap_sys_nice capability");
//!             }
//!         }
//!     }
//!     fn disable(&mut self) {
//!         unsafe {
//!             let param = sched_param { sched_priority: 0 };
//!             let result = sched_setscheduler(0, SCHED_OTHER, &param);
//!
//!             if result != 0 {
//!                 panic!("Error setting priority, you may not have cap_sys_nice capability");
//!             }
//!         }
//!     }
//! }
//! ```
//!
//! ## Putting it all together
//!
//! OK, finally we are done! Here is some example code for the main function.
//! Keep in mind, that there should be a delay between two calls of the `read` function.
//! You will not get a valid result every time, the function is called. But it
//! should be good enough to monitor the temperature of your room.
//!
//! ```
//! fn main() {
//!     let pin_number = 12;
//!     if let Ok(gpio) = Gpio::new() {
//!         if let Ok(pin) = gpio.get(pin_number) {
//!             let my_pin = MyPin::new(pin);
//!             let my_timer = MyTimer{};
//!             let my_interrupt = MyInterruptCtrl{};
//!             let mut sensor = DHTSensor::new(SensorType::DHT22, my_pin, my_timer, my_interrupt);
//!
//!             for _i in 0 .. 200 {
//!                 if let Ok(r) = sensor.read() {
//!                     println!("Temperature = {} / {} and humidity = {}",
//!                     r.temperature_celsius(),
//!                     r.temperature_fahrenheit(),
//!                     r.humidity_percent());
//!                 }
//!                 thread::sleep(Duration::from_secs(10));
//!             }
//!         } else {
//!             println!("Error: Could not get the pin!")
//!         }
//!     } else {
//!         println!("Error: Could not get the GPIOs!")
//!     }
//! }
//! ```
//! ## Dependencies
//! For this example, you need the `libc` crate, the `rppal` crate, the
//! `embedded-hal` crate, and of cause this crate!
//!
//! ```
//! [dependencies]
//! libc = "0.2.21"
//!
//! [dependencies.hal_sensor_dht]
//! path = "../hal_sensor_dht"
//! features = ["floats"]
//!
//! [dependencies.embedded-hal]
//! version = "0.2.4"
//! features = ["unproven"]
//!
//! [dependencies.rppal]
//! version = "0.11.3"
//! features = ["hal", "hal-unproven"]
//! ```


#![no_std]

extern crate embedded_hal;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{InputPin, OutputPin};

/// The maximum number of cycle loops when reading the signal line.
const MAX_CYCLES:u32 = 100_000;

/// The possible DHT errors.
///
/// `Timeout` is used when we get a timeout while expecting a pulse on the signal line.
/// `Pin` is used, when there is an error with the GPIO library.
/// `Checksum` is used, when the checksum of the data read is not correct.
pub enum DhtError {
    Timeout,
    Pin,
    Checksum,
}

/// The supported DHT sensor types.
pub enum SensorType {
    DHT11,
    DHT12,
    DHT21,
    DHT22,
}

/// Internally used for signal level. Should be part of the HAL library...
enum Level {
    Low,
    High,
}

/// Currently, the HAL library only supports `InputPin` and `OutputPin`, but we need a pin,
/// which can be reconfigured. Therefore, the GPIO type used should also provide the following
/// functions.
pub trait IoPin {
    /// Sets the pin to input mode with pullup resistor.
    fn set_input_pullup_mode(&mut self);

    /// Sets the pin to output mode.
    fn set_output_mode(&mut self);
}

/// Trait for an type which controls interrupt handling of the target architeture.
pub trait InterruptCtrl {
    /// Disable interrupts.
    ///
    /// This is done when the reading the input line is started.
    fn disable(&mut self);

    /// Enable interrupt.
    ///
    /// This is called when reading the input line is done.
    fn enable(&mut self);
}

/// This structure provides the result of the reading.
///
/// - temperature is the temperature in 1/10 degrees celsius.
/// - humidity is the humdity in 1/10 percent.
pub struct Reading {
    pub temperature : i16, // in 1/10 degree celsius
    pub humidity : u16,  // in 1/10 percent
}

impl Reading {
    /// Return the temperature in celsius as float.
    ///
    /// Note that this function is only available if the `floats` feature is selected.
    #[cfg(feature = "floats")]
    pub fn temperature_celsius(&self) -> f32 {
        f32::from(self.temperature) * 0.1
    }

    /// Return the temperature in fahrehheit as float.
    ///
    /// Note that this function is only available if the `floats` feature is selected.
    #[cfg(feature = "floats")]
    pub fn temperature_fahrenheit(&self) -> f32 {
        self.temperature_celsius() * 1.8 + 32.
    }

    /// Return the humidity as float.
    ///
    /// Note that this function is only available if the `floats` feature is selected.
    #[cfg(feature = "floats")]
    pub fn humidity_percent(&self) -> f32 {
        f32::from(self.humidity) * 0.1
    }

}

/// Interface for the DHT sensor.
pub struct DHTSensor<P, T, I>
where
    P: OutputPin + InputPin + IoPin,
    T: DelayUs<u16> + DelayMs<u16>,
    I: InterruptCtrl,
{
    pin: P,
    timer: T,
    interrupt_ctrl: I,
    sensor_type: SensorType,
}



impl<P, T, I> DHTSensor<P, T, I>
where
    P: OutputPin + InputPin + IoPin,
    T: DelayUs<u16> + DelayMs<u16>,
    I: InterruptCtrl,
{
    /// Constructs a new `DHTSensor`.
    ///
    /// The following arguments need to be provided:
    /// - `sensor` the DHT sensor type.
    /// - `p` - a pin which can act as `InputPin`, `OutputPin` and `IoPin`
    /// - `t` - a timer which implements `DelayMs` and `DelayUs`.
    /// - `i` - an interrupt controller, which is used to disable the interrupt while
    ///   reading the input line.
    pub fn new(sensor: SensorType, p: P, t: T, i: I) -> DHTSensor<P, T, I> {
        DHTSensor {
            pin: p,
            timer: t,
            interrupt_ctrl: i,
            sensor_type: sensor,
        }
    }

    /// Reads the input line and returns the temperature and humidity.
    ///
    /// Note that the calling function should obey the timing requirements for the
    /// used sensor. So do not call this function without any delay between the readings.
    pub fn read(&mut self) -> Result<Reading, DhtError> {
        let mut temp: i16;
        let hum: u16;

        match self.read_raw()  {
            Ok(data) => {
                match self.sensor_type {
                    SensorType::DHT11 => {
                        temp = i16::from(data[2]) * 10;
                        if (data[3] & 0x80) != 0 {
                            temp = -10 - temp;
                        }
                        temp += i16::from(data[3] & 0x0f);
                    }
                    SensorType::DHT12 => {
                        temp = i16::from(data[2]) * 10;
                        temp += i16::from(data[3] & 0x0f);
                        if (data[2] & 0x80) != 0 {
                            temp *= -1;
                        }
                    }
                    SensorType::DHT22 | SensorType::DHT21 => {
                        let high = i16::from(data[2] & 0x7F) << 8;
                        let low = i16::from(data[3]);
                        temp = high | low;
                        if (data[2] & 0x80) != 0 {
                            temp *= -1;
                        }
                    }
                }

                match self.sensor_type {
                    SensorType::DHT11 | SensorType::DHT12 => {
                        let high = u16::from(data[0]) * 10;
                        let low = u16::from(data[1]);
                        hum = high + low;
                    }
                    SensorType::DHT22 | SensorType::DHT21 => {
                        let high = u16::from(data[0]) << 8;
                        let low = u16::from(data[1]);
                        hum = high + low;
                    }
                }
                Ok(Reading {temperature: temp, humidity : hum,} )
            }
            Err(reason) =>
                Err(reason)
        }

    }

    /// Read data from the sensor and return the raw data.
    ///
    /// This function disables the interrupts and then starts the communication with the
    /// sensor. If the communication fails, then an Err is returned, otherwise the raw data
    /// from the sensor is returned in form of 5 data bytes.
    fn read_raw(&mut self) -> Result<[u8; 5], DhtError> {
        let mut data: [u8; 5] = [0, 0, 0, 0, 0];
        let mut cycles: [Option<u32>; 80] = [None; 80];

        // Send start signal.  See DHT datasheet for full signal diagram:
        //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

        self.interrupt_ctrl.disable();

        // Go into high impedence state to let pull-up raise data line level and
        // start the reading process.
        self.pin.set_input_pullup_mode();
        self.timer.delay_ms(1);

        // Sent a pulse to the sensor to start the sensor reading.
        self.pin.set_output_mode();

        match self.pin.set_high() {
            Err(_) => {
                return Err(DhtError::Pin);
            }
            _ => (),
        }
        self.timer.delay_ms(500);

        match self.pin.set_low() {
            Err(_) => {
                return Err(DhtError::Pin);
            }
            _ => (),
        }
        self.timer.delay_ms(20);

        // Give the sensor time to prepare the data.
        match self.sensor_type {
            SensorType::DHT22 | SensorType::DHT21 => self.timer.delay_us(1100),
            SensorType::DHT11 | SensorType::DHT12 => self.timer.delay_ms(20),
        }

        // End the start signal by setting data line high for 50 microseconds.
        // Delay a moment to let sensor pull data line low.
        self.pin.set_input_pullup_mode();
        self.timer.delay_us(50);

        // Wait for the start pulse from the sensor.
        if self.expect_pulse(Level::Low) == None {
            return Err(DhtError::Timeout);
        }
        if self.expect_pulse(Level::High) == None {
            return Err(DhtError::Timeout);
        }

        // Read 40 bits sent by the sensor.
        for i in 0..40 {
            cycles[i * 2] = self.expect_pulse(Level::Low);
            cycles[i * 2 + 1] = self.expect_pulse(Level::High);
        }
        self.interrupt_ctrl.enable();

        // Inspect the pulses:
        // - if count(high) > count(low), then a 1 has been sent.
        // - otherwise it is 0.
        // - return an error, if one of the pulses is a timeout.
        for i in 0..40 {
            if let Some(low_cycles) = cycles[2 * i] {
                if let Some(high_cycles) = cycles[2 * i + 1] {
                    data[i / 8] <<= 1;
                    if high_cycles > low_cycles {
                        data[i / 8] |= 1;
                    }
                } else {
                    return Err(DhtError::Timeout);
                }
            } else {
                return Err(DhtError::Timeout);
            }
        }

        // Compute and compare the checksum.
        let sum = u16::from(data[0]) + u16::from(data[1]) + u16::from(data[2]) + u16::from(data[3]);

        if u16::from(data[4]) == (0xff & sum) {
            Ok(data)
        } else {
            Err(DhtError::Checksum)
        }
    }

    /// Expects the input line to be at the specified level and either returns the
    /// number of loop cycles spent at this level (as `Some(count)`)
    /// or returns `None` if case of a timeout.
    fn expect_pulse(&self, level: Level) -> Option<u32> {
        let mut count: u32 = 0;
        let test_fct = match level {
            Level::Low => P::is_low,
            Level::High => P::is_high,
        };

        while let Ok(true) = test_fct(&self.pin) {
            count += 1;
            if count >= MAX_CYCLES {
                return None; // Exceeded timeout, fail.
            }
        }
        Some(count)
    }
}
