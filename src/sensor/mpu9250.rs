use crate::sensor::Sensor;
use core::convert::Infallible;
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_time::Delay;
use mpu9250::{Error, Marg, MargMeasurements, Mpu9250, SpiDevice, SpiError};

pub struct Mpu9250Sensor {
    device: Mpu9250<SpiDevice<Spi<'static, Async, Master>, Output<'static>>, Marg>,
    gpio_interrupt: ExtiInput<'static>,
    last_data: MargMeasurements<[f32; 3]>,
    on_new_data: Option<&'static dyn Fn(&MargMeasurements<[f32; 3]>)>,
}

impl Sensor<MargMeasurements<[f32; 3]>, SpawnError> for Mpu9250Sensor {
    async fn start_continuous_measurement(
        &'static mut self,
        spawner: &mut Spawner,
        callable: &'static dyn Fn(&MargMeasurements<[f32; 3]>),
    ) -> Result<(), SpawnError> {
        self.on_new_data = Some(callable);
        spawner.spawn(data_fetch_task(self))
    }

    fn get_latest_measurement(&self) -> &MargMeasurements<[f32; 3]> {
        &self.last_data
    }
}

impl Mpu9250Sensor {
    pub(crate) fn init_new(
        com: Spi<'static, Async, Master>,
        ncs: Output<'static>,
        gpio_interrupt: ExtiInput<'static>,
    ) -> Result<Self, Error<SpiError<embassy_stm32::spi::Error, Infallible>>> {
        defmt::info!("Initializing MPU9250 via SPI...");
        let device = Mpu9250::marg_default(com, ncs, &mut Delay)?;
        defmt::info!("MPU9250 initialized successfully");
        Ok(Self {
            device,
            gpio_interrupt,
            last_data: MargMeasurements {
                accel: [0.0; 3],
                gyro: [0.0; 3],
                mag: [0.0; 3],
                temp: 0.0,
            },
            on_new_data: None,
        })
    }
}

#[embassy_executor::task]
async fn data_fetch_task(self_: &'static mut Mpu9250Sensor) -> ! {
    loop {
        self_.gpio_interrupt.wait_for_falling_edge().await;
        match self_.device.all() {
            Ok(data) => self_.last_data = data,
            Err(e) => {
                defmt::error!("Failed to read sensor data: {}", e);
                continue;
            }
        }
        self_.on_new_data.unwrap()(&self_.last_data);
    }
}
