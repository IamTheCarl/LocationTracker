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

use freertos_rust::*;
extern crate alloc;

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
use lsm303dlhc::{
    Lsm303dlhc,
    AccelOdr,
    MagOdr,
};
use l3gd20::{
    L3gd20,
};

use alloc::sync::Arc;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 8_000_000;

#[derive(Clone, Copy, Debug)]
enum SensorData {
    Acceleration((i16, i16, i16)),
    Magnetic((i16, i16, i16)),
    Rotation((i16, i16, i16)),
}

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

    // Get our ports.
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    // Setup our Accelerometer.
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);
    let mut accelerometer = Lsm303dlhc::new(i2c).unwrap();

    // Setup our gyoroscope.
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
 
    let spi = hal::spi::Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        l3gd20::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut nss = gpioe.pe3
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    nss.set_high().ok();

    let mut gyoroscope = L3gd20::new(spi, nss).unwrap();

    // Setup our LEDs.

    // LED3 is pin PE9.
    let ld1 = gpioe.pe9.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut ld1 = ld1.into_active_high_switch();

    // LED4 is on pin PE8
    let ld4 = gpioe.pe8.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut ld4 = ld4.into_active_high_switch();

    // LED6 is pin PE15
    let ld6 = gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let mut ld6 = ld6.into_active_high_switch();

    // Setup our tasks to collect data from the accelerometer and gyro.
    // Give them a queue to send that data to the computation task with.
    let sensor_data_queue = Arc::new(freertos_rust::Queue::new(100).unwrap());
    let accel_queue = sensor_data_queue.clone();
    let gyro_queue = sensor_data_queue.clone();

    Task::new().name("ACEL").stack_size(512).priority(TaskPriority(1)).start(move || {
        ld1.off().ok();
        accelerometer.accel_odr(AccelOdr::Hz400).unwrap();
        accelerometer.mag_odr(MagOdr::Hz75).unwrap();

        loop {
            // TODO replace this with DMA.
            freertos_rust::CurrentTask::delay(Duration::ms(1000/400));

            let vector = accelerometer.accel();

            if let Ok(vector) = vector {
                ld1.toggle().ok();
                accel_queue.send(SensorData::Acceleration((vector.x, vector.y, vector.z)), Duration::zero()).ok();
            }

            let vector = accelerometer.mag();

            if let Ok(vector) = vector {
                ld4.toggle().ok();
                accel_queue.send(SensorData::Magnetic((vector.x, vector.y, vector.z)), Duration::zero()).ok();
            }
        }
    }).unwrap();

    Task::new().name("GYRO").stack_size(512).priority(TaskPriority(1)).start(move || {
        ld6.off().ok();
        gyoroscope.set_odr(l3gd20::Odr::Hz760).unwrap();

        loop {
            // TODO replace this with DMA.
            freertos_rust::CurrentTask::delay(Duration::ms(1000/760));

            let readings = gyoroscope.all();
            if let Ok(readings) = readings {
                ld6.toggle().ok();
                let vector = readings.gyro;
                gyro_queue.send(SensorData::Rotation((vector.x, vector.y, vector.z)), Duration::zero()).ok();
            }
        }
    }).unwrap();

    Task::new().name("DATACOMPUTE").stack_size(1024).priority(TaskPriority(2)).start(move || {
        loop {
            let reading = sensor_data_queue.receive(Duration::infinite()).ok();
            if let Some(reading) = reading {
                // info!("{:?}", reading);
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
