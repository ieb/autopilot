// Mock esp_adc_cal.h — ADC calibration stubs for native simulation.
// The sim_environment controls what analogRead() returns.

#ifndef HAL_ESP_ADC_CAL_H
#define HAL_ESP_ADC_CAL_H

#include <stdint.h>

typedef struct {
    uint32_t dummy;
} esp_adc_cal_characteristics_t;

typedef enum {
    ESP_ADC_CAL_VAL_EFUSE_VREF = 0,
    ESP_ADC_CAL_VAL_EFUSE_TP = 1,
    ESP_ADC_CAL_VAL_DEFAULT_VREF = 2,
    ESP_ADC_CAL_VAL_NOT_SUPPORTED = 3,
} esp_adc_cal_value_t;

inline esp_adc_cal_value_t esp_adc_cal_characterize(
    int unit, int atten, int width, uint32_t default_vref,
    esp_adc_cal_characteristics_t* chars) {
    (void)unit; (void)atten; (void)width; (void)default_vref; (void)chars;
    return ESP_ADC_CAL_VAL_DEFAULT_VREF;
}

// In sim mode, raw_to_voltage returns the raw value directly
// (sim_environment sets analogRead to return millivolts)
inline uint32_t esp_adc_cal_raw_to_voltage(uint32_t raw,
    const esp_adc_cal_characteristics_t* chars) {
    (void)chars;
    return raw;  // sim already provides millivolts
}

#endif // HAL_ESP_ADC_CAL_H
