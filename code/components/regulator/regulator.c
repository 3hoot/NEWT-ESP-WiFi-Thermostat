#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "nvs.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "driver/ledc.h" // For LEDC PWM control of heating and cooling pins

// User defined headers
#include "regulator.h"
#include "adc_calibration.h"

static const char *TAG = "regulator";

static bool regulator_init_done = false;   // Flag to indicate if the regulator has been initialized
static bool ntc_readout_init_done = false; // Flag to indicate if the NTC readout has been initialized
static bool pid_tune_done = false;         // Flag to indicate if the PID tuning has been completed

static bool pid_nvs_sanity_check(uint8_t pid_count)
{
    if (pid_count == 0 || pid_count > 10)
    {
        ESP_LOGE(TAG, "Invalid PID count: %d. Must be between 1 and 10.", pid_count);
        return false;
    }
    return true;
}

static bool regulator_init_done_check()
{
    if (!regulator_init_done)
    {
        ESP_LOGE(TAG, "Regulator not initialized. Call regulator_init() first.");
        return false;
    }
    return true;
}

static bool regulator_tune_done_check()
{
    if (!pid_tune_done)
    {
        ESP_LOGE(TAG, "PID tuning not completed. Call regulator_pid_tune_task() first.");
        return false;
    }
    return true;
}

static bool ntc_readout_init_done_check()
{
    if (!ntc_readout_init_done)
    {
        ESP_LOGE(TAG, "NTC readout not initialized. Call ntc_readout_init() first.");
        return false;
    }
    return true;
}

static uint32_t duty_to_pwm(int duty)
{
    if (duty < 0)
        duty = 0;
    else if (duty > 100)
        duty = 100;

    return (uint32_t)((duty / 100.0) * ((1 << PWM_BIT_WIDTH) - 1)); // fancy bit shift
}

static bool check_pid_nvs(PID_t *pid, uint8_t pid_count)
{
    // Sanity check for the number of PID structures
    if (!pid_nvs_sanity_check(pid_count))
        return false;

    // Open NVS handle for reading PID parameters
    nvs_handle_t my_handle;
    esp_err_t res = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%d) opening NVS handle!", res);
        return false;
    }

    // Check if PID parameters are initialized in NVS
    for (uint8_t i = 0; i < pid_count; i++)
    {
        // Supports only 10 PID structures for now
        PID_t *current_pid = &pid[i];
        char pid_id[PID_ID_STR_LEN]; // Buffer to hold the PID identifier string
        snprintf(pid_id, sizeof(pid_id), "PID%d", i);

        res = nvs_get_blob(my_handle, pid_id, current_pid, NULL);
        if (res != ESP_OK && res != ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Error (%d) reading PID parameters for %s from NVS!", res, pid_id);
            nvs_close(my_handle);
            return false;
        }
        else if (res == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGW(TAG, "PID parameters for %s not found in NVS!", pid_id);
            nvs_close(my_handle);
            return false;
        }

        // If we reach here, it means the PID parameters were successfully read from NVS

        // Special case:
        // If the PID parameters are all zero, we consider them uninitialized
        if (current_pid->kp == 0.0 &&
            current_pid->ki == 0.0 &&
            current_pid->kd == 0.0)
        {
            ESP_LOGW(TAG, "PID parameters for %s are uninitialized (all zeros)!", pid_id);
            nvs_close(my_handle);
            return false;
        }

        ESP_LOGI(TAG, "PID parameters for %s read from NVS!", pid_id);
    }
    nvs_close(my_handle);
    return true;
}

static void save_pid_nvs(PID_t *pid, uint8_t pid_count)
{
    // Sanity check for the number of PID structures
    if (!pid_nvs_sanity_check(pid_count))
        return;

    // Open NVS handle for writing PID parameters
    nvs_handle_t my_handle;
    esp_err_t res = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%d) opening NVS handle for writing!", res);
        return;
    }

    // Save PID parameters to NVS
    for (uint8_t i = 0; i < pid_count; i++)
    {
        PID_t *current_pid = &pid[i];
        char pid_id[PID_ID_STR_LEN]; // Buffer to hold the PID identifier string
        snprintf(pid_id, sizeof(pid_id), "PID%d", i);

        res = nvs_set_blob(my_handle, pid_id, current_pid, sizeof(PID_t));
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%d) saving PID parameters for %s to NVS!", res, pid_id);
            nvs_close(my_handle);
            return;
        }
        ESP_LOGI(TAG, "PID parameters for %s saved to NVS!", pid_id);
    }

    // Commit the changes to NVS
    res = nvs_commit(my_handle);
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Error (%d) committing changes to NVS!", res);
    else
        ESP_LOGI(TAG, "All PID parameters committed to NVS successfully!");

    nvs_close(my_handle);
}

void regulator_init(regulator_args_t *reg_args)
{
    // Initializing outputs for heating and cooling using LEDC PWM
    // For that we need one timer and two channels, one for each output
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .freq_hz = PWM_FREQUENCY_HZ,
        .duty_resolution = PWM_BIT_WIDTH,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_cooling = {
        .gpio_num = COOLING_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = COOLING_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Start with 0% duty cycle
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_cooling));

    ledc_channel_config_t ledc_channel_heating = {
        .gpio_num = HEATING_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = HEATING_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Start with 0% duty cycle
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_heating));

    ESP_LOGI(TAG, "LEDC PWM outputs for heating and cooling initialized successfully.");
    regulator_init_done = true;
}

static double get_average_ntc_temperature(regulator_args_t *reg_args)
{
    // Wait for the reading of the NTC temperature to be available
    xSemaphoreTake(reg_args->regulator->ntc_readout.lock, portMAX_DELAY);
    double ntc_temperature = 0.0;
    for (size_t i = 0; i < NTC_READOUT_QUEUE_SIZE; i++)
        ntc_temperature += reg_args->regulator->ntc_readout.input_temperature[i];
    xSemaphoreGive(reg_args->regulator->ntc_readout.lock);
    return ntc_temperature / NTC_READOUT_QUEUE_SIZE; // Calculate the average temperature
}

void regulator_pid_tune_task(void *args)
{
    if (!regulator_init_done_check())
        return;

    // Ideally this task should be started:
    // 1. After regulator_init() has been called and completed successfully.
    // 2. Before the regulator_task() is started
    // 2.1 With higher priority than the regulator_task() so that it can run first
    // 3. After the NTC readout has been initialized and is providing valid readings ==
    //    == NTC readout task needs to have higher priority!
    // But since we can never be too sure, we suspend the regulator_task() until the PID tuning is done

    // For now only tuning the kp parameter for heating and cooling, the rest will be set to 0.0

    void **args_array = (void **)args;
    TaskHandle_t regulator_task_handle = (TaskHandle_t)args_array[0];
    vTaskSuspend(regulator_task_handle);

    // Check if the NVS has valid PID parameters for both heating and cooling
    regulator_args_t *reg_args = (regulator_args_t *)args_array[1];
    PID_t pids[2] = {reg_args->regulator->pid_hot, reg_args->regulator->pid_cold};
    if (check_pid_nvs(pids, 2))
    {
        ESP_LOGI(TAG, "PID parameters found in NVS. Exiting PID tuning task ...");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "No valid PID parameters found in NVS. Starting PID tuning task ...");

    // Zero out the PID parameters for both heating and cooling
    // kp values are set to 0.1 to have some initial response
    reg_args->regulator->pid_hot.kp = 0.0;
    reg_args->regulator->pid_hot.ki = 0.0;
    reg_args->regulator->pid_hot.kd = 0.0;
    reg_args->regulator->pid_cold.kp = 0.0;
    reg_args->regulator->pid_cold.ki = 0.0;
    reg_args->regulator->pid_cold.kd = 0.0;

    // Tuning here
    bool heating = true;
    bool cooling_done = false;
    double start_temp = 0.0;
    TickType_t start_time = 0;
    bool first_run = true;

    while (!cooling_done)
    {
        double ntc_temperature = get_average_ntc_temperature(reg_args);

        if (first_run)
        {
            start_temp = ntc_temperature;
            start_time = xTaskGetTickCount();
            first_run = false;

            // Start Heating
            ESP_LOGI(TAG, "Starting Heating Test...");
            ledc_set_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL, duty_to_pwm(TUNE_HEATER_DUTY));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL);
        }

        if (heating)
        {
            if ((ntc_temperature - start_temp) >= TUNE_TEMP_RISE_THRESHOLD)
            {
                // Placeholder for heating PID tuning logic
                reg_args->regulator->pid_hot.kp = TUNE_HEATER_DUTY / TUNE_TEMP_RISE_THRESHOLD;
                ESP_LOGI(TAG, "Heating Kp: %f", reg_args->regulator->pid_hot.kp);

                // Switch to Cooling
                ledc_set_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL);

                heating = false;
                start_temp = ntc_temperature; // New baseline for cooling

                ESP_LOGI(TAG, "Starting Cooling Test...");
                ledc_set_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL, duty_to_pwm(TUNE_COOLER_DUTY));
                ledc_update_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL);
            }
        }
        else
        {
            // Cooling Logic: Look for a temperature drop
            if ((start_temp - ntc_temperature) >= TUNE_COOL_TEMP_DROP_THRESHOLD)
            {
                // Placeholder for cooling PID tuning logic
                reg_args->regulator->pid_cold.kp = TUNE_COOLER_DUTY / TUNE_COOL_TEMP_DROP_THRESHOLD;
                ESP_LOGI(TAG, "Cooling Kp: %f", reg_args->regulator->pid_cold.kp);

                ledc_set_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL);
                cooling_done = true;
            }
        }
        // Check for timeout
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(TUNE_TIMEOUT_MS))
        {
            ESP_LOGE(TAG, "PID tuning timed out! Choosing to save whatever values and exit.");
            reg_args->regulator->pid_hot.kp = TUNE_HEATER_DUTY / TUNE_TEMP_RISE_THRESHOLD;
            reg_args->regulator->pid_cold.kp = TUNE_COOLER_DUTY / TUNE_COOL_TEMP_DROP_THRESHOLD;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(REGULATOR_INTERVAL_MS)); // Crucial to prevent watchdog/task starvation
    }

    // Save and cleanup
    save_pid_nvs(pids, 2); // Save the tuned PID parameters to NVS
    vTaskResume(regulator_task_handle);
    vTaskDelete(NULL);
}

void regulator_task(void *args)
{
    regulator_args_t *reg_args = (regulator_args_t *)args;
    double setpoint = 25.0; // Define your target temperature
    double deadband = 0.5;  // Prevents oscillation around setpoint

    while (true)
    {
        // 1. Get current temperature
        xSemaphoreTake(reg_args->regulator->ntc_readout.lock, portMAX_DELAY);
        double ntc_temp = 0.0;
        for (size_t i = 0; i < NTC_READOUT_QUEUE_SIZE; i++)
            ntc_temp += reg_args->regulator->ntc_readout.input_temperature[i];
        xSemaphoreGive(reg_args->regulator->ntc_readout.lock);
        ntc_temp /= NTC_READOUT_QUEUE_SIZE;

        // 2. Calculate Error
        double error = setpoint - ntc_temp;

        // 3. Control Logic (Split-Range)
        if (error > deadband)
        {
            // HEATING PHASE
            double output = reg_args->regulator->pid_hot.kp * error;
            // Add safety clamping
            if (output > 100.0)
                output = 100.0;

            ledc_set_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL, duty_to_pwm(output));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL, 0); // Ensure cooling is off
            ledc_update_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL);
        }
        else if (error < -deadband)
        {
            // COOLING PHASE
            double output = reg_args->regulator->pid_cold.kp * fabsf(error);
            if (output > 100.0)
                output = 100.0;

            ledc_set_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL, duty_to_pwm(output));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL, 0); // Ensure heating is off
            ledc_update_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL);
        }
        else
        {
            // IDLE PHASE - Turn everything off
            ledc_set_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, HEATING_PWM_CHANNEL);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, COOLING_PWM_CHANNEL);
        }

        // 4. Rate limiting
        vTaskDelay(pdMS_TO_TICKS(REGULATOR_INTERVAL_MS));
    }
}

bool ntc_readout_init(regulator_args_t *reg_args)
{
    // Initialize ADC1 for reading the NTC thermistor value
    // (adc2 is used by the Wi-Fi driver, so we avoid using it to prevent conflicts)

    // We'll be using oneshot mode for ADC1, which is suitable for single readings with low frequency.
    // This is ideal for our use case where we don't need continuous sampling (only once per ~500ms).

    // ADC init
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_config = {.unit_id = NTC_ADC_UNIT};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc_handle));

    // ADC config
    adc_oneshot_chan_cfg_t adc_channel_config = {
        .atten = ADC_ATTEN_DB_12, // 12 dB attenuation for full-scale voltage range
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, NTC_ADC_CHANNEL, &adc_channel_config));

    // ADC calibration
    adc_cali_handle_t adc_cali_handle = NULL; // NTC (ADC1 channel 0) calibration handle
    bool calibrated = adc_calibration_init(&adc_cali_handle, NTC_ADC_UNIT, NTC_ADC_CHANNEL,
                                           ADC_ATTEN_DB_12, ADC_BITWIDTH_DEFAULT);
    if (!calibrated)
        ESP_LOGE(TAG, "ADC calibration failed for NTC (ADC1 channel 0). Readings may be inaccurate.");

    reg_args->regulator->ntc_readout.lock = xSemaphoreCreateMutex();
    if (reg_args->regulator->ntc_readout.lock == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex for NTC readout.");
        return false;
    }

    ESP_LOGI(TAG, "NTC readout initialized successfully.");
    reg_args->adc_handle = adc_handle;
    reg_args->adc_cali_handle = adc_cali_handle;
    ntc_readout_init_done = true;
    return true;
}

// Just a linear function really
static double map_ntc_voltage_to_temperature(double voltage)
{
    // Using the two calibration points to calculate the slope and intercept for linear mapping
    double slope = (NTC_CAL_TEMP2 - NTC_CAL_TEMP1) / (NTC_CAL_VOLTAGE2 - NTC_CAL_VOLTAGE1);
    double intercept = NTC_CAL_TEMP1 - slope * NTC_CAL_VOLTAGE1;

    // Calculate temperature based on the voltage reading
    double temperature = slope * voltage + intercept;
    return temperature;
}

void ntc_readout_task(void *args)
{
    // Check if the NTC readout has been initialized
    if (!ntc_readout_init_done_check())
        return;

    regulator_args_t *reg_args = (regulator_args_t *)args;
    bool first_reading = true;

    while (true)
    {
        int ntc_raw;
        int ntc_mv;
        ESP_ERROR_CHECK(adc_oneshot_read(reg_args->adc_handle, NTC_ADC_CHANNEL, &ntc_raw));
        // Assuming that the ADC is calibrated, we can convert the raw reading to voltage
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(reg_args->adc_cali_handle, ntc_raw, &ntc_mv));

        double ntc_voltage = ntc_mv / 1000.0; // Convert mV to V
        double temperature = map_ntc_voltage_to_temperature(ntc_voltage);

        if (first_reading)
        {
            // Initialize the input_temperature array with the first reading
            xSemaphoreTake(reg_args->regulator->ntc_readout.lock, portMAX_DELAY);
            for (size_t i = 0; i < NTC_READOUT_QUEUE_SIZE; i++)
            {
                reg_args->regulator->ntc_readout.input_temperature[i] = temperature;
            }
            xSemaphoreGive(reg_args->regulator->ntc_readout.lock);
            first_reading = false;
        }
        else
        {
            // Shift the readings in the input_temperature array to make room for the new reading
            xSemaphoreTake(reg_args->regulator->ntc_readout.lock, portMAX_DELAY);
            for (size_t i = 0; i < NTC_READOUT_QUEUE_SIZE - 1; i++)
            {
                reg_args->regulator->ntc_readout.input_temperature[i] = reg_args->regulator->ntc_readout.input_temperature[i + 1];
            }
            reg_args->regulator->ntc_readout.input_temperature[NTC_READOUT_QUEUE_SIZE - 1] = temperature;
            xSemaphoreGive(reg_args->regulator->ntc_readout.lock);
        }

        vTaskDelay(pdMS_TO_TICKS(NTC_READOUT_INTERVAL_MS));
    }
}