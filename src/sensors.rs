
use freertos_rust::*;
use stm32f3xx_hal as hal;
use hal::hal::{
    blocking::i2c,
    blocking::spi,
    digital::v2::OutputPin,
};
use alloc::sync::Arc;

use lsm303dlhc::{
    Lsm303dlhc,
    AccelOdr,
    MagOdr,
};

use l3gd20::L3gd20;

#[derive(Clone, Copy, Debug)]
pub enum SensorData {
    Acceleration((i16, i16, i16)),
    Magnetic((i16, i16, i16)),
    Rotation((i16, i16, i16)),
}

pub fn start_accelerometer_reading<I2C, E>(i2c: I2C, queue: Arc<freertos_rust::Queue<SensorData>>) -> Result<(), FreeRtosError>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + Send + 'static,
    E: core::fmt::Debug
{

    // TODO maybe push this error as a result rather than panic?
    let mut accelerometer = Lsm303dlhc::new(i2c)
        .expect("Failed to initalize accelerometer.");

    Task::new().name("ACEL").stack_size(512).priority(TaskPriority(1)).start(move || {
        accelerometer.accel_odr(AccelOdr::Hz100).unwrap();
        accelerometer.mag_odr(MagOdr::Hz220).unwrap();

        loop {
            freertos_rust::CurrentTask::delay(Duration::ms(5));

            // Read all acceleration data that is avalable.
            loop {
                let status = accelerometer.get_accel_status().unwrap();

                if status.new_data {
                    let vector = accelerometer.accel();

                    if let Ok(vector) = vector {
        
                        queue.send(SensorData::Acceleration((vector.x, vector.y, vector.z)), Duration::zero()).ok();
                    }
                }
                else
                {
                    break;
                }
            }

            let vector = accelerometer.mag();

            if let Ok(vector) = vector {
                queue.send(SensorData::Magnetic((vector.x, vector.y, vector.z)), Duration::zero()).ok();
            }
        }
    })?;

    Ok(())
}

pub fn start_gyroscope_sensor_reading<SPI, CS, E>(spi: SPI, cs: CS,
    queue: Arc<freertos_rust::Queue<SensorData>>) -> Result<(), FreeRtosError>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E> + Send + 'static,
    CS: OutputPin + Send + 'static,
    E: core::fmt::Debug,
{
    let mut gyoroscope = L3gd20::new(spi, cs).unwrap();

    Task::new().name("GYRO").stack_size(512).priority(TaskPriority(1)).start(move || {
        gyoroscope.set_odr(l3gd20::Odr::Hz95).unwrap();
        gyoroscope.set_scale(l3gd20::Scale::Dps2000).unwrap();
        gyoroscope.set_bandwidth(l3gd20::Bandwidth::Low).unwrap();

        loop {
            freertos_rust::CurrentTask::delay(Duration::ms(10));

            while gyoroscope.status().unwrap().new_data {
                let vector = gyoroscope.gyro();
                if let Ok(vector) = vector {
                    queue.send(SensorData::Rotation(
                            (vector.x, vector.y, vector.z)
                        ), Duration::zero()).ok();
                }
            }
        }
    }).unwrap();

    Ok(())
}