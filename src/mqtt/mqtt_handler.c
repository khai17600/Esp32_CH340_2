#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "mqtt_handler.h"
#include "../hardware/hardware.h"
#include "../network/wifi_manager.h"

#define MQTT_BROKER_URI "mqtt://10.11.50.162:1883"
#define REQUEST_TOPIC "elevator/request"
#define STATUS_TOPIC "elevator/status"
#define MAX_REQUEST_BYTES 1024
#define RESPONSE_QUEUE_LENGTH 16

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t client;
static atomic_bool mqtt_connected = false;
static QueueHandle_t response_queue;
static QueueHandle_t heartbeat_queue;
static char rx_buffer[MAX_REQUEST_BYTES + 1];
static int rx_used, rx_total;

typedef struct {
    sync_request_t request;
    sync_state_t state;
    const char *result;
    const char *error;
} status_response_t;

/* Accessed exclusively from the MQTT status task. */
static uint32_t diag_heartbeats = 0;
static uint32_t diag_responses = 0;
static uint32_t diag_enqueue_failures = 0;
static int64_t diag_last_log_ms = 0;

/* The result and error strings from elevator_sync have static storage. */
static void queue_status(const sync_request_t *r, const char *result,
                         const char *error, const sync_state_t *s)
{
    if (!r) {
        xQueueOverwrite(heartbeat_queue, s);
        return;
    }

    status_response_t response = {
        .request = *r,
        .state = *s,
        .result = result,
        .error = error,
    };
    if (xQueueSend(response_queue, &response, 0) != pdPASS) {
        ESP_LOGW(TAG, "Response queue full; request_id=%s", r->request_id);
    }
}

static void publish_status(const sync_request_t *r, const char *result,
                           const char *error, const sync_state_t *s)
{
    cJSON *json = cJSON_CreateObject();
    if (!json) return;
    cJSON_AddNumberToObject(json, "protocol_version", 2);
    cJSON_AddStringToObject(json, "robot_id", SYNC_ROBOT_ID);
    cJSON_AddStringToObject(json, "boot_id", s->boot_id);
    cJSON_AddNumberToObject(json, "status_seq", s->status_seq);
    cJSON_AddNumberToObject(json, "uptime_ms", (double)s->now_ms);
    cJSON_AddNumberToObject(json, "current_floor", s->current_floor);
    cJSON_AddNumberToObject(json, "target_floor", s->target_floor);
    cJSON_AddBoolToObject(json, "moving", s->moving);
    cJSON_AddStringToObject(json, "door_state", s->door == SYNC_CLOSED ? "CLOSED" : "OPEN");
    cJSON_AddStringToObject(json, "door_mode", s->door == SYNC_CLOSED ? "CLOSED" :
                            s->door == SYNC_HOLD ? "ROBOT_HOLD" : "AUTO");
    cJSON_AddStringToObject(json, "hold_session", s->hold_session);
    cJSON_AddStringToObject(json, "state", s->moving ? "MOVING" :
                            s->door == SYNC_CLOSED ? "DOOR_CLOSED" : "DOOR_OPEN");
    cJSON_AddStringToObject(json, "source", r ? "mqtt" : "hardware");
    cJSON_AddStringToObject(json, "result", result);
    cJSON_AddStringToObject(json, "error", error);
    if (r) {
        cJSON_AddStringToObject(json, "request_id", r->request_id);
        cJSON_AddStringToObject(json, "session_id", r->session_id);
        cJSON_AddStringToObject(json, "trip_id", r->trip_id);
        cJSON_AddStringToObject(json, "command", r->command);
        cJSON_AddNumberToObject(json, "command_seq", r->command_seq);
    }
    char *payload = cJSON_PrintUnformatted(json);
    if (payload) {
        int message_id = r
            ? esp_mqtt_client_enqueue(client, STATUS_TOPIC, payload, 0, 1, 0, true)
            : esp_mqtt_client_publish(client, STATUS_TOPIC, payload, 0, 0, 0);
        if (message_id < 0) {
            ++diag_enqueue_failures;
            ESP_LOGW(TAG, "Status send failed: rc=%d, qos=%d", message_id, r ? 1 : 0);
        }
        if (r) ++diag_responses;
        else ++diag_heartbeats;

        /* Diagnostics only: measure enqueue rate and outbox every ~10 seconds. */
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (diag_last_log_ms == 0) diag_last_log_ms = now_ms;
        if (now_ms - diag_last_log_ms >= 10000) {
            ESP_LOGI(TAG,
                     "DIAG 10s: heartbeat=%lu response=%lu send_fail=%lu outbox=%d connected=%d",
                     (unsigned long)diag_heartbeats,
                     (unsigned long)diag_responses,
                     (unsigned long)diag_enqueue_failures,
                     esp_mqtt_client_get_outbox_size(client),
                     (int)atomic_load(&mqtt_connected));
            diag_heartbeats = 0;
            diag_responses = 0;
            diag_enqueue_failures = 0;
            diag_last_log_ms = now_ms;
        }
        cJSON_free(payload);
    }
    cJSON_Delete(json);
}

static bool get_string(const cJSON *json, const char *key, char *out, size_t capacity, bool empty_ok)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, key);
    if (!cJSON_IsString(item) || !item->valuestring) return false;
    size_t size = strlen(item->valuestring);
    if (size >= capacity || (!empty_ok && !size)) return false;
    memcpy(out, item->valuestring, size + 1);
    return true;
}

static bool get_integer(const cJSON *json, const char *key, int *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, key);
    if (!cJSON_IsNumber(item) || item->valuedouble != item->valueint) return false;
    *out = item->valueint;
    return true;
}

static void handle_request(const char *data, int len)
{
    const char *end = NULL;
    cJSON *json = cJSON_ParseWithLengthOpts(data, (size_t)len + 1, &end, true);
    if (!json) { ESP_LOGW(TAG, "Invalid JSON"); return; }
    sync_request_t r = {0};
    int version = 0, seq = 0;
    bool valid = get_integer(json, "protocol_version", &version) && version == 2 &&
        get_integer(json, "command_seq", &seq) && seq > 0 &&
        get_string(json, "robot_id", r.robot_id, sizeof(r.robot_id), false) &&
        get_string(json, "request_id", r.request_id, sizeof(r.request_id), false) &&
        get_string(json, "session_id", r.session_id, sizeof(r.session_id), false) &&
        get_string(json, "trip_id", r.trip_id, sizeof(r.trip_id), true) &&
        get_string(json, "boot_id", r.boot_id, sizeof(r.boot_id), true) &&
        get_string(json, "command", r.command, sizeof(r.command), false);
    r.command_seq = (uint32_t)seq;
    const cJSON *expires = cJSON_GetObjectItemCaseSensitive(json, "expires_at_ms");
    if (strcmp(r.command, "GET_STATUS")) {
        if (!cJSON_IsNumber(expires) || expires->valuedouble < 0 ||
            expires->valuedouble > 9007199254740991.0) valid = false;
        else r.expires_at_ms = (uint64_t)expires->valuedouble;
    }
    if (!strcmp(r.command, "CALL_ELEVATOR"))
        valid = valid && get_integer(json, "destination_floor", &r.destination_floor);
    if (!strcmp(r.command, "OPEN_DOOR") || !strcmp(r.command, "CLOSE_DOOR"))
        valid = valid && get_integer(json, "expected_floor", &r.expected_floor);
    cJSON_Delete(json);
    if (!valid) { ESP_LOGW(TAG, "Invalid or legacy request; protocol v2 required"); return; }
    /* Queue full: no side effects; manager retries the same request id. */
    if (!hardware_enqueue_request(&r)) ESP_LOGW(TAG, "Request queue full; retry needed");
}

static void mqtt_event_handler(void *args, esp_event_base_t base, int32_t id, void *data)
{
    (void)args; (void)base;
    esp_mqtt_event_handle_t event = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        rx_used = rx_total = 0;
        atomic_store(&mqtt_connected, true);
        ESP_LOGI(TAG, "Connected; protocol v2; outbox=%d",
                 esp_mqtt_client_get_outbox_size(client));
        esp_mqtt_client_subscribe(client, REQUEST_TOPIC, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        atomic_store(&mqtt_connected, false);
        rx_used = rx_total = 0;
        ESP_LOGW(TAG, "Disconnected; robot hold is preserved; outbox=%d",
                 esp_mqtt_client_get_outbox_size(client));
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Subscribed: msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        /* QoS1 PUBACK; heartbeat QoS0 does not produce a matching PUBACK. */
        ESP_LOGI(TAG, "Published ACK: msg_id=%d; outbox=%d", event->msg_id,
                 esp_mqtt_client_get_outbox_size(client));
        break;
    case MQTT_EVENT_DATA:
        if (event->current_data_offset == 0) {
            rx_used = rx_total = 0;
            if (event->retain || event->topic_len != strlen(REQUEST_TOPIC) ||
                memcmp(event->topic, REQUEST_TOPIC, event->topic_len) ||
                event->total_data_len <= 0 || event->total_data_len > MAX_REQUEST_BYTES) break;
            rx_total = event->total_data_len;
        }
        if (!rx_total || event->total_data_len != rx_total ||
            event->current_data_offset != rx_used || event->data_len < 0 ||
            event->data_len > rx_total - rx_used) {
            rx_used = rx_total = 0;
            break;
        }
        memcpy(rx_buffer + rx_used, event->data, event->data_len);
        rx_used += event->data_len;
        if (rx_used == rx_total) {
            rx_buffer[rx_used] = '\0';
            handle_request(rx_buffer, rx_used);
            rx_used = rx_total = 0;
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "MQTT error; outbox=%d", esp_mqtt_client_get_outbox_size(client));
        if (event->error_handle) {
            ESP_LOGW(TAG, "MQTT error_type=%d tls_esp_err=0x%x tls_stack_err=0x%x sock_errno=%d",
                     (int)event->error_handle->error_type,
                     (unsigned)event->error_handle->esp_tls_last_esp_err,
                     (unsigned)event->error_handle->esp_tls_stack_err,
                     event->error_handle->esp_transport_sock_errno);
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGW(TAG, "MQTT connect_return_code=%d",
                         (int)event->error_handle->connect_return_code);
            }
        }
        break;
    default: break;
    }
}

static void mqtt_status_task(void *arg)
{
    (void)arg;
    bool started = false;

    while (true) {
        bool has_ip = wifi_wait_connected(0);
        if (has_ip && !started) {
            esp_err_t err = esp_mqtt_client_start(client);
            started = err == ESP_OK;
            if (!started) ESP_LOGW(TAG, "Start failed: %s", esp_err_to_name(err));
        } else if (!has_ip && started) {
            atomic_store(&mqtt_connected, false);
            esp_err_t err = esp_mqtt_client_stop(client);
            started = err != ESP_OK;
            atomic_store(&mqtt_connected, false);
            if (started) ESP_LOGW(TAG, "Stop failed: %s", esp_err_to_name(err));
        }

        status_response_t response;
        if (xQueueReceive(response_queue, &response, pdMS_TO_TICKS(100)) == pdPASS) {
            publish_status(&response.request, response.result, response.error, &response.state);
            continue;
        }

        sync_state_t state;
        if (wifi_wait_connected(0) && atomic_load(&mqtt_connected) &&
            xQueueReceive(heartbeat_queue, &state, 0) == pdPASS) {
            publish_status(NULL, "STATUS", "", &state);
        }
    }
}

void mqtt_handler_start(void)
{
    esp_mqtt_client_config_t config = {
        .broker.address.uri = MQTT_BROKER_URI,
        .network.reconnect_timeout_ms = 3000,
        .outbox.limit = 16384,
    };
    client = esp_mqtt_client_init(&config);
    if (!client) { ESP_LOGE(TAG, "MQTT init failed"); return; }
    response_queue = xQueueCreate(RESPONSE_QUEUE_LENGTH, sizeof(status_response_t));
    heartbeat_queue = xQueueCreate(1, sizeof(sync_state_t));
    ESP_ERROR_CHECK(response_queue && heartbeat_queue ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    BaseType_t task_created = xTaskCreate(mqtt_status_task, "mqtt_status", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    hardware_set_status_callback(queue_status);
}
