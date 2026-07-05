#ifndef ADC_CALIBRATION_H
#define ADC_CALIBRATION_H

#include <stdbool.h>

// ESP-IDF headers
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

/** @brief Initialize ADC calibration handle for a specific ADC unit and channel.
 *
 * Shamelessly copied from the ESP-IDF examples, but modified to fit our needs.
 *
 * @param out_handle Pointer to the ADC calibration handle that will be initialized
 * @param unit ADC unit (ADC_UNIT_1 or ADC_UNIT_2)
 * @param channel ADC channel (ADC_CHANNEL_0 to ADC_CHANNEL_10)
 * @param atten ADC attenuation (enum adc_atten_t)
 * @param bitwidth ADC bit width (enum adc_bitwidth_t)
 *
 */
bool adc_calibration_init(adc_cali_handle_t *out_handle, adc_unit_t unit, adc_channel_t channel,
                          adc_atten_t atten, adc_bitwidth_t bitwidth);

#endif // ADC_CALIBRATION_H