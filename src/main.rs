#![no_std]
#![no_main]
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use cortex_m_rt::entry;

#[allow(unused)]
use itm_logger::{
    logger_init,
    log,
    info,
    error,
    Level,
};

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

use l3gd20::L3gd20;

use alloc::sync::Arc;
use fixed_sqrt::FixedSqrt;
use fixed::types::I17F15;

// Acceleration units.
type ACU = I17F15;


#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 8_000_000;

#[global_allocator]
static ALLOCATOR: FreeRtosAllocator = FreeRtosAllocator;

mod handlers;
mod sensors;

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

    let accellerometer_i2c = hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

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
    sensors::start_accelerometer_reading(accellerometer_i2c, sensor_data_queue.clone()).unwrap();

    let gyro_queue = sensor_data_queue.clone();

    Task::new().name("GYRO").stack_size(512).priority(TaskPriority(1)).start(move || {
        gyoroscope.set_odr(l3gd20::Odr::Hz760).unwrap();
        gyoroscope.set_scale(l3gd20::Scale::Dps2000).unwrap();
        gyoroscope.set_bandwidth(l3gd20::Bandwidth::High).unwrap();

        loop {
            // TODO replace this with DMA.
            freertos_rust::CurrentTask::delay(Duration::ms(1000/380));

            if gyoroscope.status().unwrap().new_data {
                let vector = gyoroscope.gyro();
                if let Ok(vector) = vector {
                    gyro_queue.send(sensors::SensorData::Rotation((vector.x, vector.y, vector.z)), Duration::zero()).ok();
                }
            }
        }
    }).unwrap();

    Task::new().name("DATACOMPUTE").stack_size(1024).priority(TaskPriority(2)).start(move || {

        ld1.off().ok();
        ld4.off().ok();
        ld6.off().ok();

        loop {
            let reading = sensor_data_queue.receive(Duration::infinite()).ok();
            if let Some(reading) = reading {
                match reading {
                    sensors::SensorData::Acceleration(_acceleration) => {
                        ld1.toggle().ok();
                        // info!("{:?}", _acceleration);

                        let x = (ACU::from_num(_acceleration.0) / 0x7FFF) * 2;
                        let y = (ACU::from_num(_acceleration.1) / 0x7FFF) * 2;
                        let z = (ACU::from_num(_acceleration.2) / 0x7FFF) * 2;
                        let gravity = (x * x + y * y + z * z).sqrt();

                        info!("GRAVITY: {}", gravity);
                    },
                    sensors::SensorData::Magnetic(_magnetic) => {
                        ld4.toggle().ok();
                        // info!("{:?}", _magnetic);
                    },
                    sensors::SensorData::Rotation(_rotation) => {
                        ld6.toggle().ok();
                        // info!("{:?}", _rotation);
                    }
                }
            }
        }
    }).unwrap();

    FreeRtosUtils::start_scheduler();
}