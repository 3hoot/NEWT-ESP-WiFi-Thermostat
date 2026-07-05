#include <stdlib.h>
#include <stdbool.h>

// ESP-IDF headers
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"

// User defined headers
#include "adc_calibration.h"

static const char *TAG = "adc_calibration";

bool adc_calibration_init(adc_cali_handle_t *out_handle, adc_unit_t unit, adc_channel_t channel,
                          adc_atten_t atten, adc_bitwidth_t bitwidth)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t res = ESP_FAIL;

    ESP_LOGI(TAG, "Initializing linear ADC calibration for unit %d, channel %d, attenuation %d, bitwidth %d",
             unit, channel, atten, bitwidth);
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = bitwidth,
    };
    res = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create ADC calibration handle for unit %d, channel %d, attenuation %d, bitwidth %d. Error: %s",
                 unit, channel, atten, bitwidth, esp_err_to_name(res));
        return false;
    }

    *out_handle = handle;
    ESP_LOGI(TAG, "ADC calibration handle created successfully for unit %d, channel %d, attenuation %d, bitwidth %d",
             unit, channel, atten, bitwidth);
    return true;
}