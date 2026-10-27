#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "mqtt_client.h"
#include "esp_log.h"
#include "cJSON.h"

#include "mqtt_handler.h"
#include "esp_timer.h"
#include "../hardware/hardware.h"


/* =========================================================
 * MQTT CONFIG
 * ========================================================= */

#define MQTT_BROKER_URI   "mqtt://10.11.55.162:1883"

#define REQUEST_TOPIC     "elevator/request"
#define STATUS_TOPIC      "elevator/status"


/* =========================================================
 * TAG
 * ========================================================= */

static const char *TAG = "MQTT";


/* =========================================================
 * MQTT CLIENT
 * ========================================================= */

static esp_mqtt_client_handle_t client = NULL;


/* =========================================================
 * DUPLICATE PROTECTION
 * =========================================================
 *
 * Pi5 hiện tại chưa gửi request_id.
 *
 * Vì vậy tạm thời dùng:
 *
 * command + destination + thời gian
 *
 * Nếu cùng một CALL_ELEVATOR được nhận lại
 * trong khoảng thời gian này -> bỏ.
 *
 * 5 giây > thời gian animation 3 giây.
 */

#define DUPLICATE_WINDOW_MS 5000

static char last_command[32] = "";

static int last_destination = -1;

static int64_t last_request_time_ms = 0;


/* =========================================================
 * GET TIME
 * ========================================================= */

static int64_t mqtt_now_ms(void)
{
    return esp_timer_get_time() / 1000;
}


/* =========================================================
 * DUPLICATE CHECK
 * ========================================================= */

static bool is_duplicate_request(
    const char *command,
    int destination
)
{
    int64_t now =
        mqtt_now_ms();


    bool same_command =
        strcmp(
            last_command,
            command
        ) == 0;


    bool same_destination =
        last_destination ==
        destination;


    bool inside_window =
        (now - last_request_time_ms)
        <
        DUPLICATE_WINDOW_MS;


    if (
        same_command &&
        same_destination &&
        inside_window
    )
    {
        return true;
    }


    strncpy(
        last_command,
        command,
        sizeof(last_command) - 1
    );

    last_command[
        sizeof(last_command) - 1
    ] = '\0';


    last_destination =
        destination;


    last_request_time_ms =
        now;


    return false;
}


/* =========================================================
 * PUBLISH STATUS
 * ========================================================= */

static void publish_status(
    int floor,
    elevator_state_t state,
    const char *source
)
{
    if (client == NULL)
    {
        return;
    }


    const char *state_string;


    switch (state)
    {
        case ELEVATOR_STATE_MOVING:
            state_string = "MOVING";
            break;

        case ELEVATOR_STATE_ARRIVED:
            state_string = "ARRIVED";
            break;

        case ELEVATOR_STATE_DOOR_OPEN:
            state_string = "DOOR_OPEN";
            break;

        case ELEVATOR_STATE_DOOR_CLOSED:
            state_string = "DOOR_CLOSED";
            break;

        default:
            state_string = "UNKNOWN";
            break;
    }


    cJSON *json =
        cJSON_CreateObject();


    if (json == NULL)
    {
        ESP_LOGE(
            TAG,
            "Cannot create JSON object"
        );

        return;
    }


    cJSON_AddNumberToObject(
        json,
        "current_floor",
        floor
    );


    cJSON_AddStringToObject(
        json,
        "state",
        state_string
    );


    cJSON_AddStringToObject(
        json,
        "source",
        source != NULL
        ? source
        : "hardware"
    );


    char *payload =
        cJSON_PrintUnformatted(
            json
        );


    if (payload == NULL)
    {
        cJSON_Delete(json);

        return;
    }


    esp_mqtt_client_publish(
        client,
        STATUS_TOPIC,
        payload,
        0,
        1,
        0
    );


    ESP_LOGI(
        TAG,
        "STATUS -> %s",
        payload
    );


    cJSON_free(payload);

    cJSON_Delete(json);
}


/* =========================================================
 * HANDLE REQUEST
 * ========================================================= */

static void handle_request(
    const char *data,
    int len
)
{
    cJSON *json =
        cJSON_ParseWithLength(
            data,
            len
        );


    if (json == NULL)
    {
        ESP_LOGE(
            TAG,
            "Invalid JSON payload"
        );

        return;
    }


    const cJSON *command =
        cJSON_GetObjectItem(
            json,
            "command"
        );


    if (!cJSON_IsString(command))
    {
        ESP_LOGE(
            TAG,
            "Missing command"
        );

        cJSON_Delete(json);

        return;
    }


    ESP_LOGI(
        TAG,
        "Command received: %s",
        command->valuestring
    );


    /* =====================================================
     * GET STATUS
     * ===================================================== */

    if (
        strcmp(
            command->valuestring,
            "GET_STATUS"
        ) == 0
    )
    {
        int floor =
            hardware_get_current_floor();


        publish_status(
            floor,
            ELEVATOR_STATE_ARRIVED,
            "status_request"
        );


        cJSON_Delete(json);

        return;
    }


    /* =====================================================
     * CALL ELEVATOR
     * ===================================================== */

    if (
        strcmp(
            command->valuestring,
            "CALL_ELEVATOR"
        ) == 0
    )
    {
        const cJSON *dest =
            cJSON_GetObjectItem(
                json,
                "destination_floor"
            );


        if (!cJSON_IsNumber(dest))
        {
            ESP_LOGE(
                TAG,
                "Missing destination_floor"
            );

            cJSON_Delete(json);

            return;
        }


        int floor =
            dest->valueint;


        if (
            floor != 1 &&
            floor != 2
        )
        {
            ESP_LOGE(
                TAG,
                "Invalid destination floor: %d",
                floor
            );

            cJSON_Delete(json);

            return;
        }


        /*
         * ================================================
         * DUPLICATE PROTECTION
         * ================================================
         */

        if (
            is_duplicate_request(
                command->valuestring,
                floor
            )
        )
        {
            ESP_LOGW(
                TAG,
                "Duplicate CALL_ELEVATOR ignored: floor=%d",
                floor
            );

            cJSON_Delete(json);

            return;
        }


        ESP_LOGI(
            TAG,
            "Queue elevator request: floor=%d",
            floor
        );


        /*
         * QUAN TRỌNG:
         *
         * Hàm này bây giờ KHÔNG block.
         *
         * Nó chỉ đưa request vào queue.
         */
        hardware_request_floor(
            floor,
            "mqtt"
        );


        cJSON_Delete(json);

        return;
    }


    /* =====================================================
     * OPEN DOOR
     * ===================================================== */

    if (
        strcmp(
            command->valuestring,
            "OPEN_DOOR"
        ) == 0
    )
    {
        ESP_LOGI(
            TAG,
            "OPEN_DOOR received"
        );


        hardware_open_door_hold(
            "mqtt"
        );


        cJSON_Delete(json);

        return;
    }


    /* =====================================================
     * CLOSE DOOR
     * ===================================================== */

    if (
        strcmp(
            command->valuestring,
            "CLOSE_DOOR"
        ) == 0
    )
    {
        ESP_LOGI(
            TAG,
            "CLOSE_DOOR received"
        );


        hardware_close_door(
            "mqtt"
        );


        cJSON_Delete(json);

        return;
    }


    /* =====================================================
     * UNKNOWN COMMAND
     * ===================================================== */

    ESP_LOGW(
        TAG,
        "Unknown command: %s",
        command->valuestring
    );


    cJSON_Delete(json);
}


/* =========================================================
 * MQTT EVENT HANDLER
 * ========================================================= */

static void mqtt_event_handler(
    void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data
)
{
    (void)handler_args;
    (void)base;


    esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)event_data;


    switch (
        (esp_mqtt_event_id_t)event_id
    )
    {
        case MQTT_EVENT_CONNECTED:

            ESP_LOGI(
                TAG,
                "Connected to broker"
            );


            esp_mqtt_client_subscribe(
                client,
                REQUEST_TOPIC,
                1
            );


            /*
             * Báo trạng thái hiện tại.
             */
            publish_status(
                hardware_get_current_floor(),
                ELEVATOR_STATE_ARRIVED,
                "boot"
            );

            break;


        case MQTT_EVENT_DISCONNECTED:

            ESP_LOGW(
                TAG,
                "Disconnected from broker"
            );

            break;


        case MQTT_EVENT_SUBSCRIBED:

            ESP_LOGI(
                TAG,
                "Subscribed to %s",
                REQUEST_TOPIC
            );

            break;


        case MQTT_EVENT_DATA:

            handle_request(
                event->data,
                event->data_len
            );

            break;


        case MQTT_EVENT_ERROR:

            ESP_LOGE(
                TAG,
                "MQTT error event"
            );

            break;


        default:

            break;
    }
}


/* =========================================================
 * START MQTT
 * ========================================================= */

void mqtt_handler_start(void)
{
    /*
     * Hardware -> MQTT callback.
     */
    hardware_set_status_callback(
        publish_status
    );


    esp_mqtt_client_config_t mqtt_cfg =
    {
        .broker.address.uri =
            MQTT_BROKER_URI
    };


    client =
        esp_mqtt_client_init(
            &mqtt_cfg
        );


    if (client == NULL)
    {
        ESP_LOGE(
            TAG,
            "Failed to initialize MQTT client"
        );

        return;
    }


    esp_mqtt_client_register_event(
        client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );


    esp_mqtt_client_start(
        client
    );
}