#![no_std]
#![no_main]
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use cortex_m::{
    asm,
    interrupt
};

use cortex_m_rt::{
    exception,
    ExceptionFrame,
    entry,
};

use itm_logger::{
    logger_init,
    log,
    info,
    error,
    Level,
};

use core::panic::PanicInfo;

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
pub static SystemCoreClock: u32 = 8_000_000;

#[entry]
fn main() -> ! {
    // Setup logging.
    logger_init();
    // You will need to update the tpiu baudrate if you change the speed of the sysclk.
    // REMEMBER: The log functions are pretty heavy. A task needs at least 512kb of stack to use these.

    info!("Log is working.");

    // Get handles to the core peripherals.
    let _core = cortex_m::Peripherals::take().unwrap();

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

    Task::new().name("hello").stack_size(512).priority(TaskPriority(1)).start(move || {
        info!("Task started.");

        loop {
            freertos_rust::CurrentTask::delay(Duration::ms(1000));
            red.toggle().ok();
            green.toggle().ok();
        }
    }).unwrap();

    FreeRtosUtils::start_scheduler();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupt::disable();

    error!("{}", info);

    loop {
        // Get the debugger's attention.
        asm::bkpt();
    }
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
