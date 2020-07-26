#![no_std]
#![no_main]
#![feature(lang_items)]
#![feature(alloc_error_handler)]

extern crate panic_semihosting;

use cortex_m::asm;
use cortex_m_rt::{
    exception,
    ExceptionFrame,
    entry,
};

use cortex_m_log::{
    printer::Semihosting,
    modes::InterruptOk,
    log::{
        Logger,
        trick_init,
    },
};

use log::info;

use stm32f3xx_hal as hal;

use hal::{
    gpio::GpioExt,
    rcc::RccExt,
    pac::Interrupt,
};

use switch_hal::{
    OutputSwitch,
    IntoSwitch,
    ToggleableOutputSwitch,
};

use rtic::cyccnt::{Instant, U32Ext as _};
use cortex_m::peripheral::DWT;

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    #[init(schedule = [blink_red, blink_green])]
    fn init(mut cx: init::Context) {
        // Setup logging.
        let logger = Logger {
            //Uses semihosting as destination with no interrupt control.
            inner: Semihosting::<InterruptOk, _>::stdout()
                .expect("Failed to get semihosting stdout"),
            level: log::LevelFilter::Info
        };

        // Tricks the compiler into thinking the logger is a static global.
        // It dropping the logger after this will result in undefined behavior.
        unsafe {
            trick_init(&logger).ok();
        }

        info!("Log setup.");

        // Get handles to the device peripherals.
        let dp = cx.device;

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

        red.on().ok();
        green.on().ok();

        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        DWT::unlock();
        cx.core.DWT.enable_cycle_counter();

        // semantically, the monotonic timer is frozen at time "zero" during `init`
        // NOTE do *not* call `Instant::now` in this context; it will return a nonsense value
        let now = cx.start; // the start time of the system

        cx.schedule.blink_red(now + 2_000_000.cycles()).unwrap();
        // cx.schedule.blink_green(now + 4_000_000.cycles()).unwrap();
    }

    #[task]
    fn blink_red(_c: blink_red::Context) {
        info!("Blnk red!");
    }

    #[task]
    fn blink_green(_c: blink_green::Context) {
        info!("Blnk green!");
    }


    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Will put the processor to sleep until the next interrupt happens.
            // Saves us just a little power.
            // asm::wfi();
            asm::nop();
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn EXTI0();
    }
};

// #[entry]
// fn main() -> ! {
//     // Setup logging.
//     let logger = Logger {
//         //Uses semihosting as destination with no interrupt control.
//         inner: Semihosting::<InterruptOk, _>::stdout()
//             .expect("Failed to get semihosting stdout"),
//         level: log::LevelFilter::Info
//     };

//     // Tricks the compiler into thinking the logger is a static global.
//     // It dropping the logger after this will result in undefined behavior.
//     unsafe {
//         trick_init(&logger).ok();
//     }


//     // Get handles to the device peripherals.
//     let dp = hal::pac::Peripherals::take().unwrap();

//     // Get the Reset and Clock Control. This gives us access to the AMBA High Preformance Bus, needed for the GPIOs.
//     let mut rcc = dp.RCC.constrain();

//     // Get the port.
//     let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

//     // Red LED is pin PE9.
//     let red = gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
//     let mut red = red.into_active_high_switch();

//     // Green LED is pin PE15
//     let green = gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
//     let mut green = green.into_active_low_switch();

//     // let mut number = 0;

//     red.on().ok();
//     green.on().ok();

//     loop {}

//     // Task::new().name("hello").stack_size(128).priority(TaskPriority(1)).start(move || {
//     //     info!("Task started.");

//     //     loop {
//     //         freertos_rust::CurrentTask::delay(Duration::ms(1000));
//     //         red.toggle().ok();
//     //         green.toggle().ok();
//     //     }
//     // }).unwrap();

//     // FreeRtosUtils::start_scheduler();
// }

// We need a function to handle our allocation errors.
#[alloc_error_handler]
fn allocation_error_handler(_: core::alloc::Layout) -> ! {
    // We just pass our error as a panic.
    panic!("Allocation failure.");
}

#[exception]
fn DefaultHandler(irqn: i16) {
    
    let type_name = match irqn {
        -1 => "Reset",
        -2 => "NMI",
        -3 => "HardFault",
        -4 => "MemManage",
        -5 => "BusFault",
        -6 => "UsageFault",
        -10..=-7 => "Reserved",
        -11 => "SVCall",
        -12 => "Debug Monitor",
        -13 => "Reserved",
        -14 => "PendSV",
        -15 => "SysTick",
        _ => if irqn < 0 {
            "External Interrupt"
        } else {
            "Custom Exception"
        }
    };

    panic!("Default Exception Handler: ({}) - {}", irqn, type_name);
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}