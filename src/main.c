#include "esp_system.h"
#include "esp_flash.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_efuse.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pd_config.h"
#include "pd_rx.h"
#include "pd_tx.h"
#include "pd.h"

#define TAG "ESP32-PD"

void app_main()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Starting up");

    ESP_LOGI(TAG, "  * Initialize IDLE");

    /* my test setup has the 5k1 PD connected to a GPIO so i can toggle connect/disconnect */
    pd_mode(PD_MODE_IDLE);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    pd_init();

    ESP_LOGI(TAG, "  * Main loop");
    while (true)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
