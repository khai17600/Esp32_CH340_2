/* Pure state machine: exclusively owned by hardware_task. No MQTT/FreeRTOS calls. */
#include "elevator_sync.h"
#include <string.h>

static void emit(elevator_sync_t *s, const sync_request_t *r, const char *result, const char *error)
{
    ++s->state.status_seq;
    if (s->emit) s->emit(r, result, error, &s->state);
}

static void door_close(elevator_sync_t *s)
{
    s->state.door = SYNC_CLOSED;
    s->state.close_at_ms = 0;
    s->state.hold_session[0] = '\0';
}

static void door_auto(elevator_sync_t *s)
{
    s->state.door = SYNC_AUTO;
    s->state.close_at_ms = s->state.now_ms + 3000;
    s->state.hold_session[0] = '\0';
}

void elevator_sync_init(elevator_sync_t *s, int floor, const char *boot, sync_emit_fn callback)
{
    memset(s, 0, sizeof(*s));
    s->state.current_floor = (floor == 2) ? 2 : 1;
    s->state.target_floor = s->state.current_floor;
    strncpy(s->state.boot_id, boot, SYNC_ID_SIZE - 1);
    s->pending_record = -1;
    s->emit = callback;
}

void elevator_sync_report(elevator_sync_t *s, uint64_t now)
{
    s->state.now_ms = now;
    emit(s, NULL, "STATUS", "");
}

static bool same_request(const sync_request_t *a, const sync_request_t *b)
{
    return !strcmp(a->robot_id, b->robot_id) && !strcmp(a->session_id, b->session_id) &&
        !strcmp(a->trip_id, b->trip_id) && !strcmp(a->request_id, b->request_id) &&
        !strcmp(a->boot_id, b->boot_id) && !strcmp(a->command, b->command) &&
        a->command_seq == b->command_seq && a->expires_at_ms == b->expires_at_ms && a->destination_floor == b->destination_floor &&
        a->expected_floor == b->expected_floor;
}

static void finish(elevator_sync_t *s, sync_record_t *record, const char *error)
{
    record->error = error;
    record->result = error[0] ? "REJECTED" : "COMPLETED";
    emit(s, &record->request, record->result, record->error);
}

void elevator_sync_request(elevator_sync_t *s, const sync_request_t *r, uint64_t now)
{
    s->state.now_ms = now;
    if (strcmp(r->robot_id, SYNC_ROBOT_ID)) {
        emit(s, r, "REJECTED", "WRONG_ROBOT");
        return;
    }
    if (!strcmp(r->command, "GET_STATUS")) {
        /* A retry of a read returns a fresh snapshot, with the same request id. */
        emit(s, r, "COMPLETED", "");
        return;
    }
    if (strcmp(r->boot_id, s->state.boot_id)) {
        emit(s, r, "REJECTED", "BOOT_MISMATCH");
        return;
    }
    for (unsigned i = 0; i < SYNC_CACHE_SIZE; ++i) {
        sync_record_t *record = &s->cache[i];
        if (record->used && !strcmp(r->session_id, record->request.session_id) &&
            !strcmp(r->request_id, record->request.request_id)) {
            if (!same_request(r, &record->request)) emit(s, r, "REJECTED", "REQUEST_ID_CONFLICT");
            else emit(s, r, record->result, record->error);
            return;
        }
    }
    if (now > r->expires_at_ms) { emit(s, r, "REJECTED", "EXPIRED_REQUEST"); return; }
    /* Never execute an old command again after its detailed cache entry expires. */
    sync_session_t *session = NULL;
    for (unsigned i = 0; i < SYNC_SESSION_SIZE; ++i) {
        if (!strcmp(s->sessions[i].id, r->session_id)) { session = &s->sessions[i]; break; }
    }
    if (!session) {
        for (unsigned i = 0; i < SYNC_SESSION_SIZE; ++i) {
            if (!s->sessions[i].id[0]) {
                session = &s->sessions[i];
                strcpy(session->id, r->session_id);
                break;
            }
        }
    }
    if (!session) { emit(s, r, "REJECTED", "SESSION_CAPACITY"); return; }
    if (r->command_seq <= session->highest_seq) { emit(s, r, "REJECTED", "STALE_REQUEST"); return; }
    session->highest_seq = r->command_seq;
    unsigned slot = s->next_record++ % SYNC_CACHE_SIZE;
    if ((int)slot == s->pending_record) slot = s->next_record++ % SYNC_CACHE_SIZE;
    sync_record_t *record = &s->cache[slot];
    *record = (sync_record_t){.used = true, .request = *r, .result = "ACCEPTED", .error = ""};

    if (s->state.moving) { finish(s, record, "MOVING"); return; }
    if (!strcmp(r->command, "CALL_ELEVATOR")) {
        if (r->destination_floor != 1 && r->destination_floor != 2) {
            finish(s, record, "INVALID_FLOOR"); return;
        }
        if (s->state.door == SYNC_HOLD) { finish(s, record, "DOOR_HELD"); return; }
        if (r->destination_floor == s->state.current_floor) {
            if (s->state.door == SYNC_CLOSED) door_auto(s);
            finish(s, record, "");
            return;
        }
        door_close(s);
        s->state.target_floor = r->destination_floor;
        s->state.moving = true;
        s->state.move_end_ms = now + 3000;
        s->pending_record = (int)slot;
        emit(s, r, "ACCEPTED", "");
        return;
    }
    if (strcmp(r->command, "OPEN_DOOR") && strcmp(r->command, "CLOSE_DOOR")) {
        finish(s, record, "UNKNOWN_COMMAND"); return;
    }
    if (r->expected_floor != s->state.current_floor) { finish(s, record, "FLOOR_MISMATCH"); return; }
    if (s->state.door == SYNC_HOLD && strcmp(s->state.hold_session, r->session_id)) {
        finish(s, record, "DOOR_OWNED_BY_OTHER_SESSION"); return;
    }
    if (!strcmp(r->command, "OPEN_DOOR")) {
        s->state.door = SYNC_HOLD;
        s->state.close_at_ms = 0;
        strcpy(s->state.hold_session, r->session_id);
    } else {
        door_close(s);  /* Already closed is also a successful command. */
    }
    finish(s, record, "");
}

void elevator_sync_tick(elevator_sync_t *s, uint64_t now)
{
    s->state.now_ms = now;
    if (s->state.moving && now >= s->state.move_end_ms) {
        s->state.current_floor = s->state.target_floor;
        s->state.moving = false;
        door_auto(s);
        if (s->pending_record >= 0) {
            sync_record_t *record = &s->cache[s->pending_record];
            s->pending_record = -1;
            finish(s, record, "");
        } else emit(s, NULL, "STATUS", "");
    }
    if (s->state.door == SYNC_AUTO && now >= s->state.close_at_ms) {
        door_close(s);
        emit(s, NULL, "STATUS", "");
    }
}

void elevator_sync_button(elevator_sync_t *s, int floor, bool open, bool close, uint64_t now)
{
    s->state.now_ms = now;
    if (s->state.moving || s->state.door == SYNC_HOLD) return;
    if (floor == 1 || floor == 2) {
        if (floor == s->state.current_floor) door_auto(s);
        else {
            door_close(s);
            s->state.target_floor = floor;
            s->state.moving = true;
            s->state.move_end_ms = now + 3000;
        }
    } else if (open) door_auto(s);
    else if (close) door_close(s);
    else return;
    emit(s, NULL, "STATUS", "");
}
