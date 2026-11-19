/* Host test: gcc -std=c11 -Wall -Wextra -Werror -Isrc/mqtt
 * src/mqtt/elevator_sync.c test/test_elevator_sync.c -o /tmp/test_elevator_sync */
#include "elevator_sync.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const char *result, *error;
static sync_state_t snapshot;
static void capture(const sync_request_t *r, const char *res, const char *err, const sync_state_t *s)
{ (void)r; result = res; error = err; snapshot = *s; }
static sync_request_t request(unsigned seq, const char *command)
{
    sync_request_t r = {.command_seq = seq, .expires_at_ms = 100000, .destination_floor = 2, .expected_floor = 1};
    strcpy(r.robot_id, "robot3"); strcpy(r.session_id, "session1");
    strcpy(r.trip_id, "trip1"); strcpy(r.boot_id, "boot1");
    strcpy(r.command, command);
    snprintf(r.request_id, sizeof(r.request_id), "request%u", seq);
    return r;
}
int main(void)
{
    elevator_sync_t s;
    elevator_sync_init(&s, 1, "boot1", capture);
    sync_request_t close = request(1, "CLOSE_DOOR");
    elevator_sync_request(&s, &close, 0);
    assert(!strcmp(result, "COMPLETED") && snapshot.door == SYNC_CLOSED);
    sync_request_t call = request(2, "CALL_ELEVATOR");
    elevator_sync_request(&s, &call, 100);
    assert(!strcmp(result, "ACCEPTED") && snapshot.moving);
    uint64_t arrival = s.state.move_end_ms;
    elevator_sync_request(&s, &call, 1000);
    assert(!strcmp(result, "ACCEPTED") && s.state.move_end_ms == arrival);
    sync_request_t read = request(3, "GET_STATUS");
    elevator_sync_request(&s, &read, 1200);
    assert(snapshot.moving && snapshot.current_floor == 1);
    sync_request_t open = request(4, "OPEN_DOOR");
    elevator_sync_request(&s, &open, 1300);
    assert(!strcmp(error, "MOVING") && snapshot.door == SYNC_CLOSED);
    elevator_sync_tick(&s, 3100);
    assert(!strcmp(result, "COMPLETED") && !snapshot.moving && snapshot.current_floor == 2);
    elevator_sync_request(&s, &call, 3200);
    assert(!strcmp(result, "COMPLETED") && !snapshot.moving);
    open = request(5, "OPEN_DOOR"); open.expected_floor = 2;
    elevator_sync_request(&s, &open, 3300);
    assert(snapshot.door == SYNC_HOLD);
    elevator_sync_tick(&s, 10000);
    assert(s.state.door == SYNC_HOLD);
    elevator_sync_button(&s, 1, false, true, 11000);
    assert(s.state.door == SYNC_HOLD && !s.state.moving);
    call = request(6, "CALL_ELEVATOR"); call.destination_floor = 1;
    elevator_sync_request(&s, &call, 12000);
    assert(!strcmp(error, "DOOR_HELD"));
    close = request(7, "CLOSE_DOOR"); close.expected_floor = 2;
    elevator_sync_request(&s, &close, 13000);
    assert(!strcmp(result, "COMPLETED") && snapshot.door == SYNC_CLOSED);
    elevator_sync_request(&s, &call, 14000);
    assert(!strcmp(error, "DOOR_HELD") && !s.state.moving); /* Same rejected id stays rejected. */
    call = request(8, "CALL_ELEVATOR"); call.destination_floor = 1;
    elevator_sync_request(&s, &call, 15000);
    assert(s.state.moving);
    sync_request_t conflict = call; conflict.destination_floor = 2;
    elevator_sync_request(&s, &conflict, 16000);
    assert(!strcmp(error, "REQUEST_ID_CONFLICT"));
    elevator_sync_tick(&s, 18000);
    assert(s.state.current_floor == 1);
    call = request(9, "CALL_ELEVATOR"); call.destination_floor = 1;
    elevator_sync_request(&s, &call, 19000);
    assert(!strcmp(result, "COMPLETED") && !s.state.moving);
    open = request(10, "OPEN_DOOR"); open.expected_floor = 2;
    elevator_sync_request(&s, &open, 20000);
    assert(!strcmp(error, "FLOOR_MISMATCH"));
    open = request(11, "OPEN_DOOR");
    elevator_sync_request(&s, &open, 21000);
    assert(s.state.door == SYNC_HOLD);
    close = request(12, "CLOSE_DOOR"); strcpy(close.session_id, "session2");
    elevator_sync_request(&s, &close, 22000);
    assert(!strcmp(error, "DOOR_OWNED_BY_OTHER_SESSION"));
    for (unsigned i = 13; i < 40; ++i) {
        sync_request_t r = request(i, "OPEN_DOOR");
        elevator_sync_request(&s, &r, 23000 + i);
    }
    elevator_sync_request(&s, &open, 24000);
    assert(!strcmp(error, "STALE_REQUEST"));
    sync_request_t expired = request(50, "CLOSE_DOOR"); expired.expires_at_ms = 1;
    elevator_sync_request(&s, &expired, 25000);
    assert(!strcmp(error, "EXPIRED_REQUEST") && s.state.door == SYNC_HOLD);
    elevator_sync_init(&s, 1, "boot2", capture);
    elevator_sync_request(&s, &close, 0);
    assert(!strcmp(error, "BOOT_MISMATCH"));
    elevator_sync_request(&s, &read, 0);
    assert(!strcmp(result, "COMPLETED") && !strcmp(snapshot.boot_id, "boot2"));
    puts("ESP protocol state-machine tests passed");
}
