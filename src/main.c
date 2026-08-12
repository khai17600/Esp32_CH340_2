#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "network/wifi_manager.h"
#include "mqtt/mqtt_handler.h"
#include "hardware/hardware.h"
#include "storage/storage.h"

void app_main(void)
{
    wifi_init();

    storage_init();

    hardware_init();

    xTaskCreate(
        hardware_task,
        "hardware_task",
        4096,
        NULL,
        5,
        NULL);

    mqtt_handler_start();

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}