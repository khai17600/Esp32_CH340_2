#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdbool.h>

/* =========================================================
 * GPIO
 * ========================================================= */

#define LED_FLOOR_1      GPIO_NUM_18
#define LED_FLOOR_2      GPIO_NUM_19

#define BUTTON_FLOOR_1   GPIO_NUM_25
#define BUTTON_FLOOR_2   GPIO_NUM_26

#define LED_DOOR         GPIO_NUM_23
#define BUTTON_OPEN      GPIO_NUM_27
#define BUTTON_CLOSE     GPIO_NUM_32


/* =========================================================
 * Elevator state
 * ========================================================= */

typedef enum
{
    ELEVATOR_STATE_MOVING = 0,
    ELEVATOR_STATE_ARRIVED,
    ELEVATOR_STATE_DOOR_OPEN,
    ELEVATOR_STATE_DOOR_CLOSED

} elevator_state_t;


/* =========================================================
 * Door mode
 * ========================================================= */

typedef enum
{
    DOOR_MODE_CLOSED = 0,

    /*
     * Cửa được mở bằng:
     * - button OPEN
     * - tự động khi thang đến tầng
     *
     * Có timeout 3 giây.
     * Button CLOSE được phép đóng sớm.
     */
    DOOR_MODE_AUTO,

    /*
     * Cửa được mở bởi robot MQTT.
     *
     * Giữ cửa vô hạn.
     * Button CLOSE bị khóa.
     * Chỉ robot mới đóng được.
     */
    DOOR_MODE_ROBOT_HOLD

} door_mode_t;


/* =========================================================
 * Public functions
 * ========================================================= */

void hardware_init(void);

void hardware_task(void *arg);

int hardware_get_current_floor(void);


/* Gọi thang đến tầng */
void hardware_request_floor(
    int floor,
    const char *source
);


/* Robot mở và giữ cửa */
void hardware_open_door_hold(
    const char *source
);


/* Robot đóng cửa */
void hardware_close_door(
    const char *source
);


/* Callback gửi trạng thái sang MQTT */

typedef void (*hardware_status_callback_t)(
    int floor,
    elevator_state_t state,
    const char *source
);

void hardware_set_status_callback(
    hardware_status_callback_t callback
);

#endif