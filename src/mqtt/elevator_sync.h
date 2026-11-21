#ifndef ELEVATOR_SYNC_H
#define ELEVATOR_SYNC_H
#include <stdbool.h>
#include <stdint.h>

#define SYNC_ID_SIZE 40
#define SYNC_COMMAND_SIZE 24
#define SYNC_CACHE_SIZE 16
#define SYNC_SESSION_SIZE 8
#define SYNC_ROBOT_ID "robot3"

typedef struct {
    char robot_id[SYNC_ID_SIZE];
    char session_id[SYNC_ID_SIZE];
    char trip_id[SYNC_ID_SIZE];
    char request_id[SYNC_ID_SIZE];
    char boot_id[SYNC_ID_SIZE];
    char command[SYNC_COMMAND_SIZE];
    uint32_t command_seq;
    uint64_t expires_at_ms;
    int destination_floor;
    int expected_floor;
} sync_request_t;

typedef enum { SYNC_CLOSED, SYNC_AUTO, SYNC_HOLD } sync_door_t;
typedef struct {
    char boot_id[SYNC_ID_SIZE];
    uint32_t status_seq;
    uint64_t now_ms;
    int current_floor;
    int target_floor;
    bool moving;
    sync_door_t door;
    uint64_t move_end_ms;
    uint64_t close_at_ms;
    char hold_session[SYNC_ID_SIZE];
} sync_state_t;

typedef void (*sync_emit_fn)(const sync_request_t *, const char *, const char *, const sync_state_t *);
typedef struct {
    bool used;
    sync_request_t request;
    const char *result;
    const char *error;
} sync_record_t;
typedef struct {
    char id[SYNC_ID_SIZE];
    uint32_t highest_seq;
} sync_session_t;
typedef struct {
    sync_state_t state;
    sync_emit_fn emit;
    sync_record_t cache[SYNC_CACHE_SIZE];
    sync_session_t sessions[SYNC_SESSION_SIZE];
    unsigned next_record;
    unsigned next_session;
    int pending_record;
} elevator_sync_t;

void elevator_sync_init(elevator_sync_t *, int floor, const char *boot_id, sync_emit_fn);
void elevator_sync_request(elevator_sync_t *, const sync_request_t *, uint64_t now_ms);
void elevator_sync_tick(elevator_sync_t *, uint64_t now_ms);
void elevator_sync_report(elevator_sync_t *, uint64_t now_ms);
void elevator_sync_button(elevator_sync_t *, int floor, bool open, bool close, uint64_t now_ms);
#endif
