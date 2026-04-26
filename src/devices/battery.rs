use alloc::format;
use defmt::info;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::{Peri, peripherals};
use embassy_time::{Duration, Timer};
use crate::utils::DurationUtils;

// change types if needed, again this is because of embassy task not allowing generics
#[embassy_executor::task]
pub async fn battery_monitoring_task(
    mut adc_module: Adc<'static, peripherals::ADC1>,
    mut first_cell_pin: Peri<'static, peripherals::PC1>,
    mut second_cell_pin: Peri<'static, peripherals::PC0>,
) -> ! {
    info!("Battery monitoring task started");

    loop {
        // Read the ADC value from the second cell voltage divider
        // Using CYCLES112 for accurate readings from voltage divider (~1.3µs per read)
        let raw_value1 = adc_module.blocking_read(&mut first_cell_pin, SampleTime::CYCLES112);
        let raw_value2 = adc_module.blocking_read(&mut second_cell_pin, SampleTime::CYCLES112);

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
        const R_TOP: f32 = 13.0; // Top resistor in kΩ (between battery and ADC pin)
        const R_BOTTOM: f32 = 5.76; // Bottom resistor in kΩ (between ADC pin and GND)
        let battery_voltage1 = adc_voltage1 * (R_TOP + R_BOTTOM) / R_BOTTOM;
        let battery_voltage2 = adc_voltage2 * (R_TOP + R_BOTTOM) / R_BOTTOM;

        info!(
            "Battery voltage: cell 1: {} V, cell 2: {} V",
            format!("{:.1}", battery_voltage1).as_str(),
            format!("{:.1}", battery_voltage2).as_str()
        );

        2.s_timer().await;
    }
}
