#include "hardware.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_random.h"

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
static SemaphoreHandle_t tft_done;

/* Panel draws are async: the single strip buffer may be reused only after DMA finishes. */
static bool tft_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(tft_done, &woken);
    return woken == pdTRUE;
}

/* =========================================================
 * ELEVATOR STATE
 * ========================================================= */
static int current_floor = 1;
static int target_floor = 1;
static bool moving = false;
static int move_frame = 0;

/* Only hardware_task owns the state machine; MQTT only sends copied requests. */
static elevator_sync_t elevator;
static QueueHandle_t command_queue;
static sync_emit_fn status_callback;

static void forward_status(const sync_request_t *r, const char *result,
                           const char *error, const sync_state_t *state)
{
    static bool have_previous = false;
    static sync_state_t previous;
    if (!have_previous || state->moving != previous.moving ||
        state->current_floor != previous.current_floor ||
        state->target_floor != previous.target_floor) {
        if (state->moving) {
            ESP_LOGI(TAG, "[ELEVATOR] MOVING: %d -> %d", state->current_floor, state->target_floor);
        } else if (have_previous && previous.moving) {
            ESP_LOGI(TAG, "[ELEVATOR] ARRIVED: floor=%d", state->current_floor);
        } else {
            ESP_LOGI(TAG, "[ELEVATOR] AT FLOOR: %d", state->current_floor);
        }
    }
    if (!have_previous || state->door != previous.door) {
        ESP_LOGI(TAG, "[DOOR] %s | floor=%d",
                 state->door == SYNC_CLOSED ? "CLOSED" :
                 state->door == SYNC_HOLD ? "OPEN + ROBOT HOLD" : "OPEN + AUTO (3s)",
                 state->current_floor);
    }
    previous = *state;
    have_previous = true;
    /* Apply the simulated door output before acknowledging the command. */
    gpio_set_level(LED_DOOR, state->door != SYNC_CLOSED);
    gpio_set_level(LED_FLOOR_1, !state->moving && state->current_floor == 1);
    gpio_set_level(LED_FLOOR_2, !state->moving && state->current_floor == 2);
    if (status_callback) status_callback(r, result, error, state);
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
        if (xSemaphoreTake(tft_done, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            ESP_LOGE(TAG, "TFT transfer timeout");
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
        .trans_queue_depth = 4,
        .on_color_trans_done = tft_trans_done
    };

    tft_done = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(tft_done ? ESP_OK : ESP_ERR_NO_MEM);

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
    char boot_id[SYNC_ID_SIZE];
    snprintf(boot_id, sizeof(boot_id), "%08lx%08lx%08lx%08lx",
             (unsigned long)esp_random(), (unsigned long)esp_random(),
             (unsigned long)esp_random(), (unsigned long)esp_random());
    elevator_sync_init(&elevator, current_floor, boot_id, forward_status);
    command_queue = xQueueCreate(16, sizeof(sync_request_t));
    ESP_ERROR_CHECK(command_queue ? ESP_OK : ESP_ERR_NO_MEM);

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

/* Register before creating hardware_task, so the callback is immutable at runtime. */
void hardware_set_status_callback(sync_emit_fn callback)
{
    status_callback = callback;
}

bool hardware_enqueue_request(const sync_request_t *request)
{
    return command_queue && xQueueSend(command_queue, request, 0) == pdTRUE;
}

static void handle_button(const char *name, int floor, bool open, bool close, uint64_t now)
{
    ESP_LOGI(TAG, "[BUTTON] %s pressed", name);
    if (elevator.state.moving) {
        ESP_LOGW(TAG, "[BUTTON] %s ignored: elevator moving", name);
    } else if (elevator.state.door == SYNC_HOLD) {
        ESP_LOGW(TAG, "[BUTTON] %s ignored: ROBOT HOLD", name);
    } else {
        if (floor != 0 && floor == elevator.state.current_floor)
            ESP_LOGI(TAG, "[ELEVATOR] Already at floor %d; opening door", floor);
        elevator_sync_button(&elevator, floor, open, close, now);
    }
}

void hardware_task(void *arg)
{
    (void)arg;
    int last_btn1 = 0, last_btn2 = 0, last_open = 0, last_close = 0;
    uint64_t last_render = 0, last_report = 0;
    int stored_floor = current_floor;
    bool was_moving = false;
    while (1) {
        uint64_t now = esp_timer_get_time() / 1000;
        elevator_sync_tick(&elevator, now);
        sync_request_t request;
        /* Bound work per tick; buttons/animation cannot be starved by MQTT. */
        for (int i = 0; i < 4 && xQueueReceive(command_queue, &request, 0) == pdTRUE; ++i) {
            elevator_sync_request(&elevator, &request, now);
        }
        int btn1 = gpio_get_level(BUTTON_FLOOR_1);
        int btn2 = gpio_get_level(BUTTON_FLOOR_2);
        int open = gpio_get_level(BUTTON_OPEN);
        int close = gpio_get_level(BUTTON_CLOSE);
        if (btn1 && !last_btn1) handle_button("FLOOR 1", 1, false, false, now);
        if (btn2 && !last_btn2) handle_button("FLOOR 2", 2, false, false, now);
        if (open && !last_open) handle_button("OPEN", 0, true, false, now);
        if (close && !last_close) handle_button("CLOSE", 0, false, true, now);
        last_btn1 = btn1; last_btn2 = btn2; last_open = open; last_close = close;

        current_floor = elevator.state.current_floor;
        target_floor = elevator.state.target_floor;
        moving = elevator.state.moving;
        if (stored_floor != current_floor) {
            storage_save_floor(current_floor);
            stored_floor = current_floor;
        }
        if ((moving && now - last_render >= 100) || moving != was_moving) {
            move_frame = (int)(now / 100);
            tft_flush_full();
            last_render = now;
        }
        was_moving = moving;
        if (now - last_report >= 1000) {
            elevator_sync_report(&elevator, now);
            last_report = now;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
