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


    /*
     * =====================================================
     * 3. HARDWARE TASK
     * =====================================================
     *
     * Task xử lý:
     *
     * - Nút tầng 1
     * - Nút tầng 2
     * - Nút OPEN
     * - Nút CLOSE
     * - Hiệu ứng TFT khi chuyển tầng
     * - LED cửa
     * - Trạng thái thang máy
     *
     * Stack 8192 để đủ cho phần render TFT.
     */

    xTaskCreate(
        hardware_task,
        "hardware_task",
        8192,
        NULL,
        5,
        NULL
    );


    /*
     * =====================================================
     * 4. WIFI
     * =====================================================
     */

    wifi_init();


    /*
     * =====================================================
     * 5. MQTT
     * =====================================================
     */

    mqtt_handler_start();


    /*
     * =====================================================
     * MAIN LOOP
     * =====================================================
     */

    while (1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}