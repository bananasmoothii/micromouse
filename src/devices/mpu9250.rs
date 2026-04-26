use core::convert::Infallible;
use defmt::{info, trace};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::time::Hertz;
use embassy_time::Delay;
use mpu9250::{
    Error, Marg, MargMeasurements, Mpu9250, MpuConfig, SpiDevice,
    SpiError,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use core::cell::Cell;
use crate::utils::{DurationUtils, HertzUtils};

pub static LATEST_MPU: Mutex<CriticalSectionRawMutex, Cell<Option<MargMeasurements<[f32; 3]>>>> = Mutex::new(Cell::new(None));

pub struct Mpu9250Sensor {
    device: Mpu9250<SpiDevice<Spi<'static, Async, Master>, Output<'static>>, Marg>,
    // gpio_interrupt: ExtiInput<'static>,
    last_data: MargMeasurements<[f32; 3]>,
}

impl Mpu9250Sensor {
    pub(crate) fn init_new(
        com: Spi<'static, Async, Master>,
        ncs: Output<'static>,
        // gpio_interrupt: ExtiInput<'static>,
    ) -> Result<Self, Error<SpiError<spi::Error, Infallible>>> {
        info!("Initializing MPU9250 via SPI...");
        let device =
            Mpu9250::marg_with_reinit(com, ncs, &mut Delay, &mut MpuConfig::marg(), |mut spi, ncs| {
                let mut new_spi_config = spi::Config::default();
                // even though the MPU9250 supports up to 20MHz, the max kernel clock for this pin on STM32F446RE is 16MHz.
                new_spi_config.frequency = 16.mhz();
                spi.set_config(&new_spi_config).ok().map(|_| (spi, ncs))
            })?;
        // interrupts don't seem to work
        // device.interrupt_config(InterruptConfig::INT_ANYRD_CLEAR)?;
        // device.enable_interrupts(InterruptEnable::WOM_EN)?;
        // device.enable_interrupts(InterruptEnable::RAW_RDY_EN)?;
        info!("MPU9250 initialized successfully");
        Ok(Self {
            device,
            // gpio_interrupt,
            last_data: MargMeasurements {
                accel: [0.0; 3],
                gyro: [0.0; 3],
                mag: [0.0; 3],
                temp: 0.0,
            },
        })
    }

    fn on_data_ready(&mut self) {
        match self.device.all() {
            Ok(data) => {
                self.last_data = data;
                trace!("New MPU9250 data: accel={:?}, gyro={:?}, mag={:?}, temp={}", data.accel, data.gyro, data.mag, data.temp);
                LATEST_MPU.lock(|cell| cell.set(Some(data)));
            },
            Err(e) => defmt::error!("Failed to read sensor data: {}", e),
        }
    }

    pub async fn start_continuous_measurement(
        &'static mut self,
        spawner: &mut Spawner,
    ) -> Result<(), SpawnError> {
        spawner.spawn(data_fetch_task(self))
    }

    pub fn get_latest_measurement(&self) -> &MargMeasurements<[f32; 3]> {
        &self.last_data
    }
}

#[embassy_executor::task]
async fn data_fetch_task(self_: &'static mut Mpu9250Sensor) -> ! {
    loop {
        // data seems to always be ready and fresh, but always reading would block the CPU on this task
        20.ms_timer().await;
        self_.on_data_ready()
    }
}
