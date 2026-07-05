#ifndef REGULATOR_H
#define REGULATOR_H

#include <stdbool.h>
#include <stdint.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#include "driver/ledc.h"

// NVS namespace for storing PID parameters
#define PID_ID_STR_LEN 8           // Length of the PID identifier string (4 characters + null terminator)
#define NVS_NAMESPACE "PID_params" // NVS namespace for storing PID parameters

// Regulator parameters
#define DEFAULT_SETPOINT 25.0f            // Default setpoint value for the regulator (celsius)
#define REGULATOR_INTERVAL_MS 500         // Interval for regulator (and tune) control loop (milliseconds)
#define REGULATOR_MAX_TEMPERATURE 50.0f   // Maximum temperature for the regulator (and tune) (celsius)
#define TUNE_HEATER_DUTY 30               // Duty cycle for the heater during PID tuning (0 - 100)
#define TUNE_COOLER_DUTY 80               // Duty cycle for the cooler during PID tuning (0 - 100)
#define TUNE_TEMP_RISE_THRESHOLD 2.0      // Degrees Celsius to monitor for gain calc
#define TUNE_COOL_TEMP_DROP_THRESHOLD 1.0 // 1 degree drop is usually enough for cooling
#define TUNE_TIMEOUT_MS 120000            // Timeout for the tuning process (milliseconds)

// Pin definitions
#define COOLING_PWM_PIN 1
#define COOLING_PWM_CHANNEL LEDC_CHANNEL_0
#define HEATING_PWM_PIN 22
#define HEATING_PWM_CHANNEL LEDC_CHANNEL_1
#define NTC_ADC_UNIT ADC_UNIT_1
#define NTC_ADC_CHANNEL ADC_CHANNEL_0

// PWM configuration
#define PWM_FREQUENCY_HZ 1000 // Frequency of the PWM signal (Hz)
#define PWM_BIT_WIDTH 12      // Bit width of the PWM timer

// NTC readout parameters
#define NTC_READOUT_INTERVAL_MS 100 // Interval for NTC readout (milliseconds)
#define NTC_READOUT_QUEUE_SIZE 16   // Size of the moving average window
#define NTC_CAL_VOLTAGE1 0.771f
#define NTC_CAL_VOLTAGE2 1.77f
#define NTC_CAL_TEMP1 23.5f
#define NTC_CAL_TEMP2 34.0f

// Generic PID structure
typedef struct PID
{
    double kp; // Proportional gain
    double ki; // Integral gain
    double kd; // Derivative gain
} PID_t;

typedef struct ntc_readout
{
    double input_temperature[NTC_READOUT_QUEUE_SIZE]; // Array to hold the last NTC readings
    SemaphoreHandle_t lock;                           // Mutex to protect access to the input_temperature array
} ntc_readout_t;

// Task-specific regulator structure, in this case for temperature control, but can be adapted for other uses
typedef struct regulator
{
    PID_t pid_hot;             // PID parameters for heating
    PID_t pid_cold;            // PID parameters for cooling
    ntc_readout_t ntc_readout; // NTC readout structure
    double setpoint;           // Desired target value
    int output_hot;            // Output value for heating (0 - 1)
    int output_cold;           // Output value for cooling (0 - 1)
} regulator_t;

typedef struct regulator_args
{
    regulator_t *regulator;               // Pointer to the regulator structure
    adc_oneshot_unit_handle_t adc_handle; // ADC handle for reading NTC values
    adc_cali_handle_t adc_cali_handle;    // ADC calibration handle for accurate readings
} regulator_args_t;

void regulator_init(regulator_args_t *reg_args);
void regulator_pid_tune_task(void *args);
void regulator_task(void *args);

bool ntc_readout_init(regulator_args_t *reg_args);
void ntc_readout_task(void *args);

#endif // REGULATOR_H