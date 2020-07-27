#![no_std]
#![no_main]
#![feature(lang_items)]
#![feature(alloc_error_handler)]

extern crate panic_itm;

use cortex_m::asm;
use cortex_m_rt::{
    exception,
    ExceptionFrame,
    entry,
};

use cortex_m_log::{
    printer::itm::ItmSync,
    destination::Itm,
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
};

use switch_hal::{
    OutputSwitch,
    IntoSwitch,
    ToggleableOutputSwitch,
};

use freertos_rust::*; 

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 64_000_000;

#[entry]
fn main() -> ! {
    // Setup logging.
    // let logger = Logger {
    //     //Uses semihosting as destination with no interrupt control.
    //     inner: Semihosting::<InterruptOk, _>::stdout()
    //         .expect("Failed to get semihosting stdout"),
    //     level: log::LevelFilter::Info
    // };

    // Get handles to the core peripherals.
    let core = cortex_m::Peripherals::take().unwrap();

    let logger = Logger {
        //Uses semihosting as destination with no interrupt control.
        inner: ItmSync::<InterruptOk>::new(Itm::new(core.ITM)),
        level: log::LevelFilter::Info
    };

    // Tricks the compiler into thinking the logger is a static global.
    // It dropping the logger after this will result in undefined behavior.
    unsafe {
        trick_init(&logger).ok();
    }

    info!("Log is working.");

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

    red.on().ok();
    green.on().ok();

    Task::new().name("hello").stack_size(128).priority(TaskPriority(1)).start(move || {
        info!("Task started.");

        loop {
            freertos_rust::CurrentTask::delay(Duration::ms(1000));
            red.toggle().ok();
            green.toggle().ok();
        }
    }).unwrap();

    let control = cortex_m::register::control::read();
    let privledge = control.npriv();
    info!("Is privledge: {}", privledge.is_privileged());

    FreeRtosUtils::start_scheduler();
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

#[exception]
fn MemoryManagement() -> ! {
    panic!("Memory management exception.");
}

#[exception]
fn BusFault() -> !{
    panic!("Bus fault.");
}

#[exception]
fn UsageFault() -> ! {
    panic!("Usage fault.");
}

#[global_allocator]
static ALLOCATOR: FreeRtosAllocator = FreeRtosAllocator;

// We need a function to handle our allocation errors.
#[alloc_error_handler]
fn allocation_error_handler(_: core::alloc::Layout) -> ! {
    // We just pass our error as a panic.
    panic!("Allocation failure.");
}

// We mark these as no_mangle so that the linker can link the FreeRTOS code to our handler functions.
#[no_mangle]
extern "C" fn vApplicationIdleHook() {
    // Will put the processor to sleep until the next interrupt happens.
    asm::wfi();
}

#[no_mangle]
extern "C" fn vApplicationStackOverflowHook(_px_task: FreeRtosTaskHandle, pc_task_name: FreeRtosCharPtr) {
    panic!("Stack overflow in task: {:#?}", pc_task_name);
}
