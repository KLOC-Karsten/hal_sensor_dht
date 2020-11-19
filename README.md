# hal_sensor_dht
A Rust DHT driver using embedded-hal interfaces.

An interface to the DHT Digital Humidity and Temperature sensors.

## Design
The design of this crate is inspired by:
- The [Adatfruit DHT cpp](https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp) code
- The [dht-hal-drv](https://crates.io/crates/dht-hal-drv) crate
- The [dht-hal](https://crates.io/crates/dht-hal) crate
- The [dht-sensor](https://crates.io/crates/dht-sensor) crate
- The [dht22_pi](https://crates.io/crates/dht22_pi) crate

All of these libraries and also this library are basically the same,
only the interfaces are a little different.

The code has been tested with DHT22 sensor, a Raspberry Pi 3B, and the rppal.


## Issues
This library makes use of the traits provided by
the [embedded-hal](https://crates.io/crates/embedded-hal) crate.
Unfortunately, a lot of stuff is still missing in embedded-hal, such as
traits for reconfigurable GPIOs and disabling interrupts.
Therefore it should be expected that this library will be change over time.

## Usage
To create a driver for the DHT sensor, the caller have to provide the
following items:
- a GPIO pin, which implements the `InputPin` and `OutputPin` traits of `embedded-hal`.
  Additionally, it should be able to reconfigure the pin (from output to input).
  Unfortunately, this is currently not part of `embedded-hal`, so the trait from
  this crate needs to be implemented.
- a timer providing the `DelayMs` and `DelayUs` traits.
- interrupts need to be suppressed while we are reading the signal line.
  This crate provides a simple trait `InterruptCtrl` for this purpose.

## Example: Using the library on a Raspberry Pi

### Implementing the GPIO interfaces.
You can use `rppal` to control the GPIO pin.
However, a wrapper needs to be implemented because of the "orphan" rule for
implementation of external traits for external structs.

```
extern crate rppal;
use rppal::gpio::{Gpio, Mode, PullUpDown};
extern crate hal_sensor_dht;
use hal_sensor_dht::{DHTSensor, SensorType};

struct MyPin(rppal::gpio::IoPin);

impl MyPin {
    pub fn new(pin: rppal::gpio::Pin) -> MyPin {
        MyPin(pin.into_io(Mode::Input))
    }
}

impl InputPin for MyPin {
    type Error = <rppal::gpio::IoPin as InputPin>::Error;
    fn is_high(&self) -> Result<bool, <rppal::gpio::IoPin as InputPin>::Error> {
        Ok(self.0.is_high())
    }
    fn is_low(&self) -> Result<bool, <rppal::gpio::IoPin as InputPin>::Error> {
        Ok(self.0.is_low())
    }
}

impl OutputPin for MyPin {
    type Error = <rppal::gpio::IoPin as OutputPin>::Error;
    fn set_high(&mut self) -> Result<(), <rppal::gpio::IoPin as OutputPin>::Error> {
        Ok(self.0.set_high())
    }
    fn set_low(&mut self) -> Result<(), <rppal::gpio::IoPin as OutputPin>::Error> {
        Ok(self.0.set_low())
    }
}

impl hal_sensor_dht::IoPin for MyPin {
    fn set_input_pullup_mode(&mut self) {
        self.0.set_mode(Mode::Input);
        self.0.set_pullupdown(PullUpDown::PullUp);
    }
    fn set_output_mode(&mut self) {
        self.0.set_mode(Mode::Output);
    }
}
```

### Implementing the Delay interfaces.
The `DelayMs` is no problem, but microsecond delay is. However, only need a
a tiny delay, therefore we use write and read operation to produce such small
delay.

```
use std::thread;
use std::time::Duration;
use rppal::gpio::{Gpio, Mode, PullUpDown};

use std::ptr::read_volatile;
use std::ptr::write_volatile;
struct MyTimer {}

impl DelayUs<u16> for MyTimer {
    fn delay_us(&mut self, t:u16) {
        let mut i = 0;
        unsafe {
            while read_volatile(&mut i) < t {
                write_volatile(&mut i, read_volatile(&mut i) + 1);
            }
        }
    }
}

impl DelayMs<u16> for MyTimer {
    fn delay_ms(&mut self, ms: u16) {
        thread::sleep(Duration::from_millis(ms.into()));
    }
}
```

### Disabling Interrupts.

You will use `sched_setscheduler` from the `libc` crate for this purpose.
This is good enough for reading the sensor data.

```
extern crate libc;
use libc::sched_param;
use libc::sched_setscheduler;
use libc::SCHED_FIFO;
use libc::SCHED_OTHER;

struct MyInterruptCtrl {}

impl hal_sensor_dht::InterruptCtrl for MyInterruptCtrl {
    fn enable(&mut self) {
        unsafe {
            let param = sched_param { sched_priority: 32 };
            let result = sched_setscheduler(0, SCHED_FIFO, &param);

            if result != 0 {
                panic!("Error setting priority, you may not have cap_sys_nice capability");
            }
        }
    }
    fn disable(&mut self) {
        unsafe {
            let param = sched_param { sched_priority: 0 };
            let result = sched_setscheduler(0, SCHED_OTHER, &param);

            if result != 0 {
                panic!("Error setting priority, you may not have cap_sys_nice capability");
            }
        }
    }
}
```

### Putting it all together

OK, finally we are done! Here is some example code for the main function.
Keep in mind, that there should be a delay between two calls of the `read` function.
You will not get a valid result every time, the function is called. But it
should be good enough to monitor the temperature of your room.

```
fn main() {
    let pin_number = 12;
    if let Ok(gpio) = Gpio::new() {
        if let Ok(pin) = gpio.get(pin_number) {
            let my_pin = MyPin::new(pin);
            let my_timer = MyTimer{};
            let my_interrupt = MyInterruptCtrl{};
            let mut sensor = DHTSensor::new(SensorType::DHT22, my_pin, my_timer, my_interrupt);

            for _i in 0 .. 200 {
                if let Ok(r) = sensor.read() {
                    println!("Temperature = {} / {} and humidity = {}",
                    r.temperature_celsius(),
                    r.temperature_fahrenheit(),
                    r.humidity_percent());
                }
                thread::sleep(Duration::from_secs(10));
            }
        } else {
            println!("Error: Could not get the pin!")
        }
    } else {
        println!("Error: Could not get the GPIOs!")
    }
}
```
### Dependencies
For this example, you need the `libc` crate, the `rppal` crate, the
`embedded-hal` crate, and of cause this crate!

```
[dependencies]
libc = "0.2.21"

[dependencies.hal_sensor_dht]
path = "../hal_sensor_dht"
features = ["floats"]

[dependencies.embedded-hal]
version = "0.2.4"
features = ["unproven"]

[dependencies.rppal]
version = "0.11.3"
features = ["hal", "hal-unproven"]
```
