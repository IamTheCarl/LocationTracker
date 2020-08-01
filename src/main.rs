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
    prelude::*,
};

use switch_hal::{
    OutputSwitch,
    IntoSwitch,
    ToggleableOutputSwitch,
};

use freertos_rust::*; 

use lsm303dlhc::Lsm303dlhc;

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

    // Get access to the clocks.
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Setup our Accelerometer.
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);
    let mut accelerometer = Lsm303dlhc::new(i2c).unwrap();

    // Get the port.
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    // LED3 is pin PE9.
    let ld1 = gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut ld1 = ld1.into_active_high_switch();

    // LED6 is pin PE15
    let ld6 = gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut ld6 = ld6.into_active_high_switch();

    ld1.on().ok();
    ld6.off().ok();

    Task::new().name("hello").stack_size(1024).priority(TaskPriority(1)).start(move || {
        info!("Task started.");

        loop {
            freertos_rust::CurrentTask::delay(Duration::ms(100));
            ld1.toggle().ok();
            ld6.toggle().ok();

            match accelerometer.accel() {
                Ok(vector) => {
                    info!("{:?}", vector);
                },
                Err(error) => {
                    error!("Failed to get acceleration: {:?}", error);
                }
            }
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

#[no_mangle]
extern "C" fn vApplicationIdleHook() {
    // Will put the processor to sleep until the next interrupt happens.
    asm::wfi();
}

#[no_mangle]
extern "C" fn vApplicationStackOverflowHook(_px_task: FreeRtosTaskHandle, pc_task_name: FreeRtosCharPtr) {
    panic!("Stack overflow in task: {:#?}", pc_task_name);
}
