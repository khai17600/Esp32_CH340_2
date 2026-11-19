#ifndef HARDWARE_H
#define HARDWARE_H
#include <stdbool.h>
#include "mqtt/elevator_sync.h"
#define LED_FLOOR_1 GPIO_NUM_18
#define LED_FLOOR_2 GPIO_NUM_19
#define BUTTON_FLOOR_1 GPIO_NUM_25
#define BUTTON_FLOOR_2 GPIO_NUM_26
#define LED_DOOR GPIO_NUM_23
#define BUTTON_OPEN GPIO_NUM_27
#define BUTTON_CLOSE GPIO_NUM_32
void hardware_init(void);
void hardware_task(void *arg);
bool hardware_enqueue_request(const sync_request_t *request);
void hardware_set_status_callback(sync_emit_fn callback);
#endif
