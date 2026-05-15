use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask, INIT_MUSIC};
use crate::utils::{DurationUtils, HertzUtils};
use alloc::format;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{ADC1, PC0, PC1, PC4};
use embassy_stm32::{Peri, peripherals};
use micromath::F32Ext;

/// Current battery voltage in millivolts, updated by battery_monitoring_task.
pub static BATTERY_VOLTAGE_MV: AtomicU32 = AtomicU32::new(0);

static SAMPLE_CYCLES: SampleTime = SampleTime::CYCLES144;

pub async fn start_battery_monitoring(
    spawner: &Spawner,
    mut adc_module: Adc<'static, ADC1>,
    mut first_cell_pin: Option<Peri<'static, PC0>>,
    mut second_cell_pin: Peri<'static, PC1>,
    en_pin: Peri<'static, PC4>,
    mut leds: [Output<'static>; 4],
) {
    let mut en_pin = Output::new(en_pin, Level::Low, Speed::Low);

    // do one blocking loop

    if let Some(first_cell_pin) = first_cell_pin.as_mut() {
        handle_two_cells(
            &mut adc_module,
            first_cell_pin,
            &mut second_cell_pin,
            &mut en_pin,
            &mut leds,
        )
            .await;
    } else {
        handle_combined_cells(
            &mut adc_module,
            &mut second_cell_pin,
            &mut en_pin,
            &mut leds,
        )
            .await;
    }

    spawner
        .spawn(battery_monitoring_task(
            adc_module,
            first_cell_pin,
            second_cell_pin,
            en_pin,
            leds,
        ))
        .unwrap();
}

// change types if needed, again this is because of embassy task not allowing generics
#[embassy_executor::task]
async fn battery_monitoring_task(
    mut adc_module: Adc<'static, ADC1>,
    first_cell_pin: Option<Peri<'static, PC0>>,
    mut second_cell_pin: Peri<'static, PC1>,
    mut en_pin: Output<'static>,
    mut leds: [Output<'static>; 4],
) -> ! {
    info!("Battery monitoring task started");

    if let Some(mut first_cell_pin) = first_cell_pin {
        loop {
            handle_two_cells(
                &mut adc_module,
                &mut first_cell_pin,
                &mut second_cell_pin,
                &mut en_pin,
                &mut leds,
            )
                .await;
        }
    } else {
        loop {
            handle_combined_cells(
                &mut adc_module,
                &mut second_cell_pin,
                &mut en_pin,
                &mut leds,
            )
                .await;
        }
    }
}

const R_TOP: f32 = 100.0; // Top resistor in kΩ (between battery and ADC pin)
const R_BOTTOM: f32 = 51.0; // Bottom resistor in kΩ (between ADC pin and GND)

async fn handle_combined_cells(
    adc_module: &mut Adc<'static, ADC1>,
    second_cell_pin: &mut Peri<'static, PC1>,
    en_pin: &mut Output<'static>,
    leds: &mut [Output<'static>; 4],
) {
    let raw_value = adc_module.blocking_read(second_cell_pin, SAMPLE_CYCLES);
    let adc_voltage = (raw_value as f32 / 4095.0) * 3.3;
    let battery_voltage = adc_voltage * (R_TOP + R_BOTTOM) / R_BOTTOM;

    info!(
        "Battery voltage: {} V (~{} V per cell)",
        format!("{:.1}", battery_voltage).as_str(),
        format!("{:.1}", battery_voltage / 2.0).as_str(),
    );

    BATTERY_VOLTAGE_MV.store((battery_voltage * 1000.0) as u32, Ordering::Relaxed);
    let pct = battery_percent(battery_voltage);
    update_leds(leds, pct);
    if 1.0 < battery_voltage && battery_voltage <= 7.1 {
        shutdown(en_pin);
    } else {
        en_pin.set_high();
    }

    wait_or_alert(leds, pct).await;
}

async fn handle_two_cells(
    adc_module: &mut Adc<'static, ADC1>,
    first_cell_pin: &mut Peri<'static, PC0>,
    second_cell_pin: &mut Peri<'static, PC1>,
    en_pin: &mut Output<'static>,
    leds: &mut [Output<'static>; 4],
) {
    // Read the ADC value from the second cell voltage divider
    // Using CYCLES112 for accurate readings from voltage divider (~1.3µs per read)
    let raw_value1 = adc_module.blocking_read(first_cell_pin, SAMPLE_CYCLES);
    let raw_value2 = adc_module.blocking_read(second_cell_pin, SAMPLE_CYCLES);

    // Convert raw ADC value (0-4095) to voltage at the ADC pin (0-3.3V)
    let adc_voltage1 = (raw_value1 as f32 / 4095.0) * 3.3;
    let adc_voltage2 = (raw_value2 as f32 / 4095.0) * 3.3 - adc_voltage1;

    // Convert back to actual battery voltage using voltage divider ratio
    // Voltage divider formula: V_in = V_out * (R_top + R_bottom) / R_bottom
    //
    // For a 2S LiPo battery (max 8.4V when fully charged):
    // - Using 10kΩ (top) + 4.7kΩ (bottom) gives ratio of ~3.13
    // - Max voltage at ADC pin: 8.4V / 3.13 = 2.68V (safe, below 3.3V)
    //
    // Adjust R_TOP and R_BOTTOM if you use different resistor values!
    let battery_voltage1 = adc_voltage1 * (R_TOP + R_BOTTOM) / R_BOTTOM;
    let battery_voltage2 = adc_voltage2 * (R_TOP + R_BOTTOM) / R_BOTTOM;

    info!(
        "Battery voltage: cell 1: {} V, cell 2: {} V",
        format!("{:.1}", battery_voltage1).as_str(),
        format!("{:.1}", battery_voltage2).as_str()
    );

    let total_voltage = battery_voltage1 + battery_voltage2;
    BATTERY_VOLTAGE_MV.store((total_voltage * 1000.0) as u32, Ordering::Relaxed);
    let pct = battery_percent(total_voltage);
    update_leds(leds, pct);
    if (0.5 < battery_voltage1 && battery_voltage1 <= 3.3)
        || (0.5 < battery_voltage2 && battery_voltage2 <= 3.3)
    {
        shutdown(en_pin);
    } else {
        en_pin.set_high();
    }

    wait_or_alert(leds, pct).await;
}

fn battery_percent(total_voltage: f32) -> f32 {
    const LIPO_2S_MIN: f32 = 7.0;
    const LIPO_2S_MAX: f32 = 8.4;
    ((total_voltage - LIPO_2S_MIN) / (LIPO_2S_MAX - LIPO_2S_MIN) * 100.0).clamp(0.0, 100.0)
}

fn update_leds(leds: &mut [Output<'static>; 4], pct: f32) {
    for (i, led) in leds.iter_mut().enumerate() {
        let threshold = (i as f32 + 1.0) * 25.0 - (if i == 3 { 5.0 } else { 0.0 }); // 25, 50, 75, 95
        if pct >= threshold {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}

async fn wait_or_alert(leds: &mut [Output<'static>; 4], pct: f32) {
    if pct < 25.0 {
        let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
            freq: 2000.hz(),
            duration: 200.ms(),
        });
        for _ in 0..5 {
            leds[0].set_high();
            500.ms_timer().await;
            leds[0].set_low();
            500.ms_timer().await;
        }
    } else {
        2.s_timer().await;
    }
}

fn shutdown(en_pin: &mut Output<'static>) {
    info!("Shutting down power output");
    for freq in INIT_MUSIC.into_iter().rev() {
        BUZZER_CHANNEL
            .try_send(BuzzerTask {
                freq: freq.hz(),
                duration: 100.ms(),
            })
            .ok();
    }
    en_pin.set_low();
}
