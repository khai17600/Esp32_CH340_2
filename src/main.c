#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "network/wifi_manager.h"
#include "mqtt/mqtt_handler.h"
#include "hardware/hardware.h"
#include "storage/storage.h"


/* =========================================================
 * APP MAIN
 * ========================================================= */

void app_main(void)
{
    /*
     * =====================================================
     * 1. STORAGE
     * =====================================================
     *
     * Phải init storage trước hardware.
     *
     * Hardware sẽ đọc current_floor từ storage
     * để sau khi reset / nhấn EN, TFT vẫn hiển thị
     * đúng tầng cuối cùng.
     */

    storage_init();


    /*
     * =====================================================
     * 2. HARDWARE + TFT
     * =====================================================
     *
     * hardware_init() sẽ:
     *
     * - Khởi tạo GPIO
     * - Khởi tạo TFT ST7789V
     * - Cấp phát framebuffer TFT
     * - Đọc tầng hiện tại từ storage
     * - Hiển thị đúng tầng hiện tại
     *
     * Quan trọng:
     * Không có hiệu ứng di chuyển khi boot.
     */

    hardware_init();


    /* Register MQTT status callback before starting the sole state owner. */
    wifi_init();
    mqtt_handler_start();
    configASSERT(xTaskCreate(hardware_task, "hardware_task", 8192, NULL, 5, NULL) == pdPASS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
