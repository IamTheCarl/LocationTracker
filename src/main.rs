#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m_rt::entry;

use cortex_m_log::printer::Semihosting;
use cortex_m_log::modes::InterruptOk;
use cortex_m_log::log::{Logger, trick_init};
use log::info;

use stm32f3xx_hal as hal;

use hal::gpio::GpioExt;
use hal::rcc::RccExt;

use switch_hal::{
    OutputSwitch,
    IntoSwitch,
    ToggleableOutputSwitch
};

#[entry]
fn main() -> ! {
    // Setup logging.
    let logger = Logger {
        //Uses semihosting as destination with no interrupt control.
        inner: Semihosting::<InterruptOk, _>::stdout().expect("Failed to get semihosting stdout"),
        level: log::LevelFilter::Info
    };

    // Tricks the compiler into thinking the logger is a static global.
    // It dropping the logger after this will result in undefined behavior.
    unsafe {
        let _ = trick_init(&logger);
    }


    // Get handles to the device peripherals.
    let dp = hal::pac::Peripherals::take().unwrap();

    // Get the Reset and Clock Control. This gives us access to the AMBA High Preformance Bus, needed for the GPIOs.
    let mut rcc = dp.RCC.constrain();

    // Get the port.
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    // Red LED is pin PE9.
    let red = gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut red = red.into_active_high_switch();

    // Green LED is pin PE15
    let green = gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut green = green.into_active_low_switch();

    let mut number = 0;

    red.on().ok();
    green.on().ok();

    loop {
        info!("Some print with newline! {}", number);
        number  += 1;

        red.toggle().ok();
        green.toggle().ok();
        cortex_m::asm::delay(8_000_000);
    }
}