#include "hardware.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/* =========================================================
 * VARIABLES
 * ========================================================= */

static int current_floor = 1;

static bool moving = false;


/*
 * Door state
 */
static door_mode_t door_mode = DOOR_MODE_CLOSED;


/*
 * Thời điểm cửa AUTO phải đóng.
 *
 * Dùng tick của FreeRTOS thay vì vTaskDelay().
 *
 * Nhờ vậy hardware_task vẫn chạy liên tục
 * và có thể nhận nút CLOSE.
 */
static TickType_t door_auto_close_tick = 0;


/*
 * Callback MQTT
 */
static hardware_status_callback_t status_callback = NULL;


/*
 * Nguồn của lần di chuyển hiện tại.
 *
 * Ví dụ:
 *
 * "button"
 * "mqtt"
 */
static const char *movement_source = "hardware";


/* =========================================================
 * STATUS CALLBACK
 * ========================================================= */

void hardware_set_status_callback(
    hardware_status_callback_t callback
)
{
    status_callback = callback;
}


/* =========================================================
 * SEND STATUS
 * ========================================================= */

static void send_status(
    elevator_state_t state,
    const char *source
)
{
    if (status_callback != NULL)
    {
        status_callback(
            current_floor,
            state,
            source
        );
    }
}


/* =========================================================
 * FLOOR LED
 * ========================================================= */

static void update_floor_led(void)
{
    if (current_floor == 1)
    {
        gpio_set_level(
            LED_FLOOR_1,
            1
        );

        gpio_set_level(
            LED_FLOOR_2,
            0
        );
    }
    else
    {
        gpio_set_level(
            LED_FLOOR_1,
            0
        );

        gpio_set_level(
            LED_FLOOR_2,
            1
        );
    }
}


/* =========================================================
 * GET CURRENT FLOOR
 * ========================================================= */

int hardware_get_current_floor(void)
{
    return current_floor;
}


/* =========================================================
 * DOOR CLOSE
 * ========================================================= */

static void door_close_internal(
    const char *source
)
{
    gpio_set_level(
        LED_DOOR,
        0
    );

    door_mode = DOOR_MODE_CLOSED;

    door_auto_close_tick = 0;

    printf(
        "[DOOR] CLOSED | source=%s | floor=%d\n",
        source,
        current_floor
    );

    send_status(
        ELEVATOR_STATE_DOOR_CLOSED,
        source
    );
}


/* =========================================================
 * OPEN DOOR AUTO
 *
 * Dùng cho:
 *
 * 1. Button OPEN
 * 2. Tự động khi thang đến tầng
 *
 * Cửa sáng 3 giây.
 *
 * Button CLOSE có thể đóng sớm.
 * ========================================================= */

static void door_open_auto(
    const char *source
)
{
    gpio_set_level(
        LED_DOOR,
        1
    );

    door_mode = DOOR_MODE_AUTO;

    /*
     * Lưu thời điểm phải đóng cửa.
     */
    door_auto_close_tick =
        xTaskGetTickCount()
        +
        pdMS_TO_TICKS(3000);

    printf(
        "[DOOR] OPEN | AUTO | source=%s | floor=%d\n",
        source,
        current_floor
    );

    send_status(
        ELEVATOR_STATE_DOOR_OPEN,
        source
    );
}


/* =========================================================
 * ROBOT OPEN + HOLD
 *
 * Không có timeout.
 * ========================================================= */

void hardware_open_door_hold(
    const char *source
)
{
    /*
     * Nếu đang mở rồi thì chỉ cần
     * chuyển sang ROBOT_HOLD.
     */

    gpio_set_level(
        LED_DOOR,
        1
    );

    door_mode = DOOR_MODE_ROBOT_HOLD;

    /*
     * Không đặt timeout.
     */
    door_auto_close_tick = 0;

    printf(
        "[DOOR] OPEN + HOLD | source=%s | floor=%d\n",
        source,
        current_floor
    );

    send_status(
        ELEVATOR_STATE_DOOR_OPEN,
        source
    );
}


/* =========================================================
 * ROBOT CLOSE
 *
 * Robot có quyền đóng cửa khi
 * đang ROBOT_HOLD.
 * ========================================================= */

void hardware_close_door(
    const char *source
)
{
    /*
     * Robot luôn được phép đóng.
     */

    if (door_mode == DOOR_MODE_CLOSED)
    {
        printf(
            "[DOOR] Already closed\n"
        );

        return;
    }

    door_close_internal(
        source
    );
}


/* =========================================================
 * REQUEST FLOOR
 * ========================================================= */

void hardware_request_floor(
    int floor,
    const char *source
)
{
    if (floor != 1 && floor != 2)
    {
        printf(
            "[ELEVATOR] Invalid floor: %d\n",
            floor
        );

        return;
    }


    /*
     * Nếu đang di chuyển thì bỏ yêu cầu mới.
     */

    if (moving)
    {
        printf(
            "[ELEVATOR] Already moving\n"
        );

        return;
    }


    /*
     * Nếu đang ở tầng được yêu cầu.
     *
     * Không cần di chuyển.
     */
    if (floor == current_floor)
    {
        printf(
            "[ELEVATOR] Already at floor %d\n",
            floor
        );

        /*
         * Nếu cửa đang đóng thì tự mở 3 giây.
         */
        if (door_mode == DOOR_MODE_CLOSED)
        {
            door_open_auto(
                source
            );
        }

        return;
    }


    movement_source = source;

    moving = true;


    printf(
        "\n========================================\n"
    );

    printf(
        "[ELEVATOR] MOVING: %d -> %d\n",
        current_floor,
        floor
    );


    /*
     * Nếu cửa đang mở AUTO thì đóng.
     *
     * Nếu robot đang HOLD thì không tự ý đóng
     * trong trường hợp này.
     */
    if (door_mode == DOOR_MODE_AUTO)
    {
        door_close_internal(
            "auto"
        );
    }


    /*
     * Khi thang di chuyển:
     * tắt LED cửa.
     */

    gpio_set_level(
        LED_DOOR,
        0
    );

    door_mode = DOOR_MODE_CLOSED;


    /*
     * Tắt LED tầng trong lúc di chuyển.
     */

    gpio_set_level(
        LED_FLOOR_1,
        0
    );

    gpio_set_level(
        LED_FLOOR_2,
        0
    );


    /*
     * Báo MOVING ngay lập tức.
     */

    send_status(
        ELEVATOR_STATE_MOVING,
        source
    );


    /*
     * Mô phỏng thời gian di chuyển = 3 giây.
     *
     * Đây là sleep của hardware_task,
     * nhưng nút CLOSE không cần xử lý
     * trong thời gian này vì cửa đã đóng.
     */

    vTaskDelay(
        pdMS_TO_TICKS(3000)
    );


    /*
     * Cập nhật tầng.
     */

    current_floor = floor;


    /*
     * Hiển thị LED tầng.
     */

    update_floor_led();


    moving = false;


    printf(
        "[ELEVATOR] ARRIVED at floor %d\n",
        current_floor
    );


    /*
     * Báo ARRIVED.
     */

    send_status(
        ELEVATOR_STATE_ARRIVED,
        source
    );


    /*
     * Sau khi đến tầng:
     * tự động mở cửa 3 giây.
     */

    door_open_auto(
        "auto"
    );


    printf(
        "========================================\n"
    );
}


/* =========================================================
 * HARDWARE INIT
 * ========================================================= */

void hardware_init(void)
{
    /*
     * ==============================
     * LED
     * ==============================
     */

    gpio_config_t led_cfg =
    {
        .pin_bit_mask =
            (1ULL << LED_FLOOR_1) |
            (1ULL << LED_FLOOR_2) |
            (1ULL << LED_DOOR),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    gpio_config(
        &led_cfg
    );


    /*
     * ==============================
     * BUTTON
     * ==============================
     *
     * Bạn đang dùng điện trở kéo xuống
     * bên ngoài.
     *
     * Vì vậy:
     *
     * Không bấm = 0
     * Bấm = 1
     */

    gpio_config_t button_cfg =
    {
        .pin_bit_mask =
            (1ULL << BUTTON_FLOOR_1) |
            (1ULL << BUTTON_FLOOR_2) |
            (1ULL << BUTTON_OPEN) |
            (1ULL << BUTTON_CLOSE),

        .mode =
            GPIO_MODE_INPUT,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    gpio_config(
        &button_cfg
    );


    /*
     * ==============================
     * DEFAULT STATE
     * ==============================
     *
     * current_floor được giữ ở 1.
     *
     * Lưu ý:
     * Nếu bạn đang dùng phần lưu NVS từ code
     * trước đó thì có thể thay dòng này bằng
     * giá trị đã đọc từ NVS.
     */

    current_floor = 1;

    moving = false;

    door_mode =
        DOOR_MODE_CLOSED;

    door_auto_close_tick = 0;


    /*
     * LED ban đầu:
     *
     * tầng 1 sáng
     * tầng 2 tắt
     * cửa tắt
     */

    update_floor_led();

    gpio_set_level(
        LED_DOOR,
        0
    );


    printf(
        "\n"
        "========================================\n"
        "       ELEVATOR HARDWARE READY\n"
        "========================================\n"
        "Current floor : %d\n"
        "Door          : CLOSED\n"
        "========================================\n",
        current_floor
    );
}


/* =========================================================
 * HARDWARE TASK
 * ========================================================= */

void hardware_task(void *arg)
{
    int last_btn1 = 0;
    int last_btn2 = 0;
    int last_open = 0;
    int last_close = 0;


    while (1)
    {
        /*
         * ============================================
         * READ BUTTONS
         * ============================================
         */

        int btn1 =
            gpio_get_level(
                BUTTON_FLOOR_1
            );

        int btn2 =
            gpio_get_level(
                BUTTON_FLOOR_2
            );

        int btn_open =
            gpio_get_level(
                BUTTON_OPEN
            );

        int btn_close =
            gpio_get_level(
                BUTTON_CLOSE
            );


        /*
         * ============================================
         * DOOR AUTO TIMEOUT
         * ============================================
         *
         * Chỉ AUTO mới có timeout.
         *
         * ROBOT_HOLD không bao giờ vào đây.
         */

        if (
            door_mode == DOOR_MODE_AUTO
            &&
            door_auto_close_tick != 0
        )
        {
            TickType_t now =
                xTaskGetTickCount();


            if (
                now >=
                door_auto_close_tick
            )
            {
                door_close_internal(
                    "auto"
                );
            }
        }


        /*
         * ============================================
         * BUTTON CONTROL
         * ============================================
         */

        if (!moving)
        {
            /*
             * ----------------------------------------
             * FLOOR 1
             * ----------------------------------------
             */

            if (
                btn1 == 1
                &&
                last_btn1 == 0
            )
            {
                hardware_request_floor(
                    1,
                    "button"
                );
            }


            /*
             * ----------------------------------------
             * FLOOR 2
             * ----------------------------------------
             */

            if (
                btn2 == 1
                &&
                last_btn2 == 0
            )
            {
                hardware_request_floor(
                    2,
                    "button"
                );
            }


            /*
             * ----------------------------------------
             * OPEN BUTTON
             * ----------------------------------------
             */

            if (
                btn_open == 1
                &&
                last_open == 0
            )
            {
                /*
                 * Button OPEN:
                 *
                 * AUTO mode.
                 *
                 * Sau 3 giây tự đóng.
                 */

                if (
                    door_mode ==
                    DOOR_MODE_CLOSED
                )
                {
                    door_open_auto(
                        "button"
                    );
                }
            }


            /*
             * ----------------------------------------
             * CLOSE BUTTON
             * ----------------------------------------
             *
             * Đây là phần quan trọng.
             *
             * CLOSE chỉ hoạt động:
             *
             * AUTO
             *
             * Không hoạt động:
             *
             * ROBOT_HOLD
             */

            if (
                btn_close == 1
                &&
                last_close == 0
            )
            {
                if (
                    door_mode ==
                    DOOR_MODE_AUTO
                )
                {
                    /*
                     * Đóng ngay lập tức.
                     */

                    door_close_internal(
                        "button"
                    );
                }
                else if (
                    door_mode ==
                    DOOR_MODE_ROBOT_HOLD
                )
                {
                    /*
                     * Không làm gì.
                     *
                     * Robot đang giữ cửa.
                     */

                    printf(
                        "[DOOR] CLOSE button ignored "
                        "(ROBOT HOLD)\n"
                    );
                }
            }
        }


        /*
         * ============================================
         * SAVE BUTTON STATES
         * ============================================
         */

        last_btn1 =
            btn1;

        last_btn2 =
            btn2;

        last_open =
            btn_open;

        last_close =
            btn_close;


        /*
         * ============================================
         * TASK PERIOD
         * ============================================
         */

        vTaskDelay(
            pdMS_TO_TICKS(20)
        );
    }
}