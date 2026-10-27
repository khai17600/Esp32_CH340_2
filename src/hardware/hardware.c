#include "hardware.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "storage/storage.h"

/* =========================================================
 * LOG
 * ========================================================= */
static const char *TAG = "HARDWARE";

/* =========================================================
 * TFT ST7789V
 *
 * 240 x 320
 *
 * SCK   -> GPIO14
 * MOSI  -> GPIO13
 * CS    -> GPIO15
 * DC    -> GPIO21
 * RST   -> GPIO22
 * LED   -> 3.3V
 * ========================================================= */
#define TFT_SCLK       GPIO_NUM_14
#define TFT_MOSI       GPIO_NUM_13
#define TFT_MISO       (-1)

#define TFT_CS         GPIO_NUM_15
#define TFT_DC         GPIO_NUM_21
#define TFT_RST        GPIO_NUM_22

#define TFT_SPI_HOST   SPI2_HOST

#define TFT_WIDTH      240
#define TFT_HEIGHT     320

#define TFT_PIXEL_CLOCK_HZ    (20 * 1000 * 1000)

/* =========================================================
 * SMALL TFT BUFFER (9.6 KB DMA RAM)
 * ========================================================= */
#define TFT_BUFFER_LINES   20
#define TFT_BUFFER_PIXELS  (TFT_WIDTH * TFT_BUFFER_LINES)

/* =========================================================
 * RGB565 BẢNG MÀU TỐI GIẢN
 * ========================================================= */
#define COLOR_BLACK       0x0000
#define COLOR_WHITE       0xFFFF
#define COLOR_DIM_WHITE   0x4208

/* =========================================================
 * TFT GLOBAL
 * ========================================================= */
static esp_lcd_panel_io_handle_t tft_io = NULL;
static esp_lcd_panel_handle_t tft_panel = NULL;
static uint16_t *tft_buffer = NULL;

/* =========================================================
 * ELEVATOR STATE
 * ========================================================= */
static int current_floor = 1;
static int target_floor = 1;
static bool moving = false;
static int move_frame = 0;

/* =========================================================
 * DOOR
 * ========================================================= */
static door_mode_t door_mode = DOOR_MODE_CLOSED;
static TickType_t door_auto_close_tick = 0;

/* =========================================================
 * CALLBACK
 * ========================================================= */
static hardware_status_callback_t status_callback = NULL;

/* =========================================================
 * MOVEMENT SOURCE
 * ========================================================= */
static const char *movement_source = "hardware";

/* =========================================================
 * SEND STATUS
 * ========================================================= */
static void send_status(elevator_state_t state, const char *source)
{
    if (status_callback != NULL)
    {
        status_callback(current_floor, state, source);
    }
}

/* =========================================================
 * SMALL BUFFER PIXEL
 * ========================================================= */
static inline void buffer_set_pixel(int x, int y, int y_start, uint16_t color)
{
    if (tft_buffer == NULL) return;
    if (x < 0 || x >= TFT_WIDTH) return;
    if (y < y_start || y >= y_start + TFT_BUFFER_LINES) return;

    tft_buffer[(y - y_start) * TFT_WIDTH + x] = color;
}

/* =========================================================
 * BUFFER CLEAR
 * ========================================================= */
static void buffer_clear(uint16_t color)
{
    if (tft_buffer == NULL) return;
    for (int i = 0; i < TFT_BUFFER_PIXELS; i++)
    {
        tft_buffer[i] = color;
    }
}

/* =========================================================
 * FILL RECT INTO SMALL BUFFER
 * ========================================================= */
static void buffer_fill_rect(int x, int y, int w, int h, uint16_t color, int y_start)
{
    if (tft_buffer == NULL) return;

    int x0 = (x < 0) ? 0 : x;
    int y0 = (y < y_start) ? y_start : y;
    int x1 = (x + w > TFT_WIDTH) ? TFT_WIDTH : (x + w);
    int y1 = (y + h > y_start + TFT_BUFFER_LINES) ? (y_start + TFT_BUFFER_LINES) : (y + h);

    if (x0 >= x1 || y0 >= y1) return;

    for (int yy = y0; yy < y1; yy++)
    {
        int row_idx = (yy - y_start) * TFT_WIDTH;
        for (int xx = x0; xx < x1; xx++)
        {
            tft_buffer[row_idx + xx] = color;
        }
    }
}

/* =========================================================
 * VẼ SỐ TẦNG 1 & 2 CHUẨN ĐẸP CÂN GIỮA
 * ========================================================= */
static void draw_floor_digit(int floor, int cx, int cy, uint16_t color, int y_start)
{
    int t = 16;  // Độ dày thanh
    int w = 70;  // Chiều rộng
    int h = 120; // Chiều cao

    int x0 = cx - w / 2;
    int y0 = cy - h / 2;

    if (floor == 1)
    {
        // Thân dọc chính giữa
        buffer_fill_rect(cx - t / 2, y0, t, h, color, y_start);
        // Móc bên TRÁI trên
        buffer_fill_rect(cx - t / 2 - 16, y0 + 4, 16, t, color, y_start);
        // Chân đế ngang DƯỚI
        buffer_fill_rect(cx - 30, y0 + h - t, 60, t, color, y_start);
    }
    else if (floor == 2)
    {
        // 1. Thanh ngang TRÊN CÙNG
        buffer_fill_rect(x0, y0, w, t, color, y_start);
        // 2. Nét dọc PHẢI TRÊN
        buffer_fill_rect(x0 + w - t, y0, t, h / 2, color, y_start);
        // 3. Thanh ngang GIỮA
        buffer_fill_rect(x0, y0 + (h / 2) - (t / 2), w, t, color, y_start);
        // 4. Nét dọc TRÁI DƯỚI
        buffer_fill_rect(x0, y0 + h / 2, t, h / 2, color, y_start);
        // 5. Thanh ngang ĐÁY DƯỚI CÙNG
        buffer_fill_rect(x0, y0 + h - t, w, t, color, y_start);
    }
}

/* =========================================================
 * VẼ MŨI TÊN TAM GIÁC PHÍA TRÊN
 * ========================================================= */
static void draw_arrow(int cx, int cy, int size, bool up, uint16_t color, int y_start)
{
    for (int dy = 0; dy < size; dy++)
    {
        int y = up ? (cy - size + 1 + dy) : (cy + size - 1 - dy);
        int half_w = dy;
        buffer_fill_rect(cx - half_w, y, 2 * half_w + 1, 1, color, y_start);
    }
}

/* =========================================================
 * RENDER ONE SCREEN STRIP (TỐI GIẢN - CÂN TÂM)
 * ========================================================= */
static void render_strip(int y_start)
{
    buffer_clear(COLOR_BLACK);

    int center_x = TFT_WIDTH / 2;   // 120 (chính giữa ngang)
    int digit_cy = TFT_HEIGHT / 2;  // 160 (chính giữa dọc)
    int arrow_cy = 45;              // Mũi tên nằm khoảng trên

    // 1. Vẽ số tầng màu trắng ở giữa
    draw_floor_digit(current_floor, center_x, digit_cy, COLOR_WHITE, y_start);

    // 2. Mũi tên hiển thị khi di chuyển
    if (moving)
    {
        bool going_up = (target_floor > current_floor);
        bool blink = (move_frame % 4) < 3;
        uint16_t arrow_col = blink ? COLOR_WHITE : COLOR_DIM_WHITE;

        draw_arrow(center_x, arrow_cy, 24, going_up, arrow_col, y_start);
    }
}

/* =========================================================
 * FLUSH FULL SCREEN
 * ========================================================= */
static void tft_flush_full(void)
{
    if (tft_panel == NULL || tft_buffer == NULL) return;

    for (int y = 0; y < TFT_HEIGHT; y += TFT_BUFFER_LINES)
    {
        render_strip(y);

        int y_end = y + TFT_BUFFER_LINES;
        if (y_end > TFT_HEIGHT) y_end = TFT_HEIGHT;

        esp_err_t err = esp_lcd_panel_draw_bitmap(tft_panel, 0, y, TFT_WIDTH, y_end, tft_buffer);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "TFT draw failed: %s", esp_err_to_name(err));
            return;
        }
    }
}

/* =========================================================
 * SHOW CURRENT FLOOR
 * ========================================================= */
static void tft_show_floor(int floor)
{
    current_floor = floor;
    moving = false;
    tft_flush_full();
}

/* =========================================================
 * TFT INIT
 * ========================================================= */
static void tft_init(void)
{
    ESP_LOGI(TAG, "Initializing ST7789V...");

    spi_bus_config_t bus_config = {
        .sclk_io_num = TFT_SCLK,
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = TFT_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_WIDTH * TFT_BUFFER_LINES * sizeof(uint16_t)
    };

    ESP_ERROR_CHECK(spi_bus_initialize(TFT_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = TFT_CS,
        .dc_gpio_num = TFT_DC,
        .pclk_hz = TFT_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TFT_SPI_HOST, &io_config, &tft_io));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TFT_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(tft_io, &panel_config, &tft_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(tft_panel));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_lcd_panel_init(tft_panel));
    vTaskDelay(pdMS_TO_TICKS(100));

    /* CẤU HÌNH XOAY MÀN HÌNH THEO HƯỚNG NHÌN TỪ CHÂN CẮM DÂY XUỐNG */
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(tft_panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(tft_panel, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(tft_panel, true));

    tft_buffer = heap_caps_malloc(TFT_BUFFER_PIXELS * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (tft_buffer == NULL)
    {
        ESP_LOGE(TAG, "Cannot allocate TFT buffer!");
        return;
    }

    ESP_LOGI(TAG, "ST7789 initialized: %dx%d", TFT_WIDTH, TFT_HEIGHT);
}

/* =========================================================
 * HARDWARE INIT
 * ========================================================= */
void hardware_init(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_FLOOR_1) | (1ULL << LED_FLOOR_2) | (1ULL << LED_DOOR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_FLOOR_1) | (1ULL << BUTTON_FLOOR_2) | (1ULL << BUTTON_OPEN) | (1ULL << BUTTON_CLOSE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));

    current_floor = storage_load_floor();
    if (current_floor != 1 && current_floor != 2)
    {
        current_floor = 1;
        storage_save_floor(current_floor);
    }

    target_floor = current_floor;
    moving = false;
    door_mode = DOOR_MODE_CLOSED;
    door_auto_close_tick = 0;

    gpio_set_level(LED_DOOR, 0);
    gpio_set_level(LED_FLOOR_1, (current_floor == 1) ? 1 : 0);
    gpio_set_level(LED_FLOOR_2, (current_floor == 2) ? 1 : 0);

    tft_init();

    if (tft_buffer != NULL)
    {
        tft_show_floor(current_floor);
    }

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "ELEVATOR HARDWARE READY");
    ESP_LOGI(TAG, "Stored floor: %d", current_floor);
    ESP_LOGI(TAG, "Target floor: %d", target_floor);
    ESP_LOGI(TAG, "Moving: false");
    ESP_LOGI(TAG, "================================");
}

int hardware_get_current_floor(void)
{
    return current_floor;
}

void hardware_set_status_callback(hardware_status_callback_t callback)
{
    status_callback = callback;
}

static void door_close_internal(const char *source)
{
    gpio_set_level(LED_DOOR, 0);
    door_mode = DOOR_MODE_CLOSED;
    door_auto_close_tick = 0;

    printf("[DOOR] CLOSED | source=%s | floor=%d\n", source, current_floor);
    send_status(ELEVATOR_STATE_DOOR_CLOSED, source);
}

static void door_open_auto(const char *source)
{
    gpio_set_level(LED_DOOR, 1);
    door_mode = DOOR_MODE_AUTO;
    door_auto_close_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);

    printf("[DOOR] OPEN | AUTO | source=%s | floor=%d\n", source, current_floor);
    send_status(ELEVATOR_STATE_DOOR_OPEN, source);
}

void hardware_open_door_hold(const char *source)
{
    gpio_set_level(LED_DOOR, 1);
    door_mode = DOOR_MODE_ROBOT_HOLD;
    door_auto_close_tick = 0;

    printf("[DOOR] OPEN + HOLD | source=%s | floor=%d\n", source, current_floor);
    send_status(ELEVATOR_STATE_DOOR_OPEN, source);
}

void hardware_close_door(const char *source)
{
    if (door_mode == DOOR_MODE_CLOSED)
    {
        printf("[DOOR] Already closed\n");
        return;
    }

    door_close_internal(source);
}

void hardware_request_floor(int floor, const char *source)
{
    if (floor != 1 && floor != 2)
    {
        printf("[ELEVATOR] Invalid floor: %d\n", floor);
        return;
    }

    if (moving)
    {
        printf("[ELEVATOR] Already moving\n");
        return;
    }

    if (door_mode == DOOR_MODE_ROBOT_HOLD)
    {
        ESP_LOGW(TAG, "Cua dang duoc ROBOT HOLD, tu choi di chuyen (nguon: %s)", source);
        return;
    }

    if (floor == current_floor)
    {
        printf("[ELEVATOR] Already at floor %d\n", floor);
        if (door_mode == DOOR_MODE_CLOSED)
        {
            door_open_auto(source);
        }
        tft_show_floor(current_floor);
        return;
    }

    movement_source = source;
    target_floor = floor;
    moving = true;

    printf("\n========================================\n");
    printf("[ELEVATOR] MOVING: %d -> %d\n", current_floor, target_floor);

    if (door_mode == DOOR_MODE_AUTO)
    {
        door_close_internal("auto");
    }

    gpio_set_level(LED_DOOR, 0);
    door_mode = DOOR_MODE_CLOSED;
    door_auto_close_tick = 0;

    gpio_set_level(LED_FLOOR_1, 0);
    gpio_set_level(LED_FLOOR_2, 0);

    send_status(ELEVATOR_STATE_MOVING, source);

    /* =====================================================
     * ANIMATION DI CHUYỂN CHUẨN XÁC ĐÚNG 3.0 GIÂY
     * ===================================================== */
    TickType_t last_wake_time = xTaskGetTickCount();

    for (int frame = 0; frame < 30; frame++)
    {
        move_frame = frame;

        if (tft_buffer != NULL)
        {
            tft_flush_full();
        }

        // Tự động bù trừ thời gian render SPI để chu kỳ luôn đúng 100ms
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }

    /* =====================================================
     * ARRIVED
     * ===================================================== */
    current_floor = target_floor;
    moving = false;
    storage_save_floor(current_floor);

    if (tft_buffer != NULL)
    {
        tft_show_floor(current_floor);
    }

    gpio_set_level(LED_FLOOR_1, (current_floor == 1) ? 1 : 0);
    gpio_set_level(LED_FLOOR_2, (current_floor == 2) ? 1 : 0);

    printf("[ELEVATOR] ARRIVED at floor %d\n", current_floor);
    send_status(ELEVATOR_STATE_ARRIVED, source);

    door_open_auto("auto");
    printf("========================================\n");
}

/* =========================================================
 * HARDWARE TASK
 * ========================================================= */
void hardware_task(void *arg)
{
    (void)arg;

    int last_btn1 = 0;
    int last_btn2 = 0;
    int last_open = 0;
    int last_close = 0;

    while (1)
    {
        int btn1 = gpio_get_level(BUTTON_FLOOR_1);
        int btn2 = gpio_get_level(BUTTON_FLOOR_2);
        int btn_open = gpio_get_level(BUTTON_OPEN);
        int btn_close = gpio_get_level(BUTTON_CLOSE);

        if (door_mode == DOOR_MODE_AUTO && door_auto_close_tick != 0)
        {
            if (xTaskGetTickCount() >= door_auto_close_tick)
            {
                door_close_internal("auto");
            }
        }

        if (!moving)
        {
            if (btn1 == 1 && last_btn1 == 0)
            {
                hardware_request_floor(1, "button");
            }

            if (btn2 == 1 && last_btn2 == 0)
            {
                hardware_request_floor(2, "button");
            }

            if (btn_open == 1 && last_open == 0)
            {
                if (door_mode == DOOR_MODE_CLOSED)
                {
                    door_open_auto("button");
                }
            }

            if (btn_close == 1 && last_close == 0)
            {
                if (door_mode == DOOR_MODE_AUTO)
                {
                    door_close_internal("button");
                }
                else if (door_mode == DOOR_MODE_ROBOT_HOLD)
                {
                    printf("[DOOR] CLOSE ignored (ROBOT HOLD)\n");
                }
            }
        }

        last_btn1 = btn1;
        last_btn2 = btn2;
        last_open = btn_open;
        last_close = btn_close;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}