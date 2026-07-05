#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// ESP-IDF headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h" // System initialization and management
#include "nvs_flash.h"  // NVS initialization

// User defined headers
#include "network.h"

static const char *TAG = "main";

// --- Function prototypes ---
static void nvs_init();

void app_main(void)
{
    nvs_init(); // before initializing Wi-Fi, initialize NVS (Non-Volatile Storage) to store Wi-Fi credentials
    wifi_init();

    // Wait for connection
    if (!wifi_connected(portMAX_DELAY))
        esp_restart(); // Restart the device if not connected within the timeout

    while (true)
    {
        ESP_LOGI(TAG, "Hello world!");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop delay
    }
}

static void nvs_init()
{
    ESP_LOGI(TAG, "Initializing NVS flash");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}