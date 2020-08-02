
use freertos_rust::*;
use stm32f3xx_hal as hal;
use hal::hal::blocking::i2c::{Write, WriteRead};
use alloc::sync::Arc;

use lsm303dlhc::{
    Lsm303dlhc,
    AccelOdr,
    MagOdr,
};

#[derive(Clone, Copy, Debug)]
pub enum SensorData {
    Acceleration((i16, i16, i16)),
    Magnetic((i16, i16, i16)),
    Rotation((i16, i16, i16)),
}

pub fn start_accelerometer_reading<I2C, E>(i2c: I2C, queue: Arc<freertos_rust::Queue<SensorData>>) -> Result<(), FreeRtosError>
where
    I2C: WriteRead<Error = E> + Write<Error = E> + Send + 'static,
    E: core::fmt::Debug
{
    let mut accelerometer = Lsm303dlhc::new(i2c)
        .expect("Failed to initalize accelerometer.");

    Task::new().name("ACEL").stack_size(512).priority(TaskPriority(1)).start(move || {
        accelerometer.accel_odr(AccelOdr::Hz200).unwrap();
        accelerometer.mag_odr(MagOdr::Hz220).unwrap();

        loop {
            // TODO replace this with DMA.
            freertos_rust::CurrentTask::delay(Duration::ms(1000/220));

            let vector = accelerometer.accel();

            if let Ok(vector) = vector {

                queue.send(SensorData::Acceleration((vector.x, vector.y, vector.z)), Duration::zero()).ok();
            }

            let vector = accelerometer.mag();

            if let Ok(vector) = vector {
                queue.send(SensorData::Magnetic((vector.x, vector.y, vector.z)), Duration::zero()).ok();
            }
        }
    })?;

    Ok(())
}

// TODO move sensor prosessing into here.