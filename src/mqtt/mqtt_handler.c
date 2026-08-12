#include <stdio.h>
#include <string.h>

#include "mqtt_client.h"
#include "esp_log.h"
#include "cJSON.h"

#include "mqtt_handler.h"
#include "../hardware/hardware.h"


#define MQTT_BROKER_URI \
    "mqtt://192.168.1.9:1883"

#define REQUEST_TOPIC \
    "elevator/request"

#define STATUS_TOPIC \
    "elevator/status"


static const char *TAG = "MQTT";

static esp_mqtt_client_handle_t client = NULL;


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
            "Cannot create JSON"
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
        source
    );


    char *payload =
        cJSON_PrintUnformatted(
            json
        );


    if (payload == NULL)
    {
        ESP_LOGE(
            TAG,
            "Cannot create JSON payload"
        );

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


    cJSON_free(
        payload
    );


    cJSON_Delete(
        json
    );
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


        int destination_floor =
            dest->valueint;


        ESP_LOGI(
            TAG,
            "CALL_ELEVATOR -> floor %d",
            destination_floor
        );


        hardware_request_floor(
            destination_floor,
            "mqtt"
        );


        cJSON_Delete(json);

        return;
    }


    /* =====================================================
     * OPEN DOOR + HOLD
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
            "OPEN_DOOR received from ROBOT"
        );


        /*
         * Robot mở cửa và giữ.
         *
         * Button CLOSE trên ESP
         * sẽ bị khóa.
         */

        hardware_open_door_hold(
            "mqtt_robot"
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
            "CLOSE_DOOR received from ROBOT"
        );


        /*
         * Robot được phép đóng cửa
         * kể cả khi đang ROBOT_HOLD.
         */

        hardware_close_door(
            "mqtt_robot"
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
    esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)
        event_data;


    switch (
        (esp_mqtt_event_id_t)
        event_id
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


            ESP_LOGI(
                TAG,
                "Subscribed to %s",
                REQUEST_TOPIC
            );


            /*
             * Báo vị trí hiện tại.
             *
             * Không báo MOVING.
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
                "Subscribed successfully"
            );

            break;


        case MQTT_EVENT_DATA:

            ESP_LOGI(
                TAG,
                "MQTT request received"
            );


            ESP_LOGI(
                TAG,
                "Payload: %.*s",
                event->data_len,
                event->data
            );


            handle_request(
                event->data,
                event->data_len
            );

            break;


        case MQTT_EVENT_ERROR:

            ESP_LOGE(
                TAG,
                "MQTT error"
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
     * Hardware thay đổi trạng thái
     * -> gọi publish_status()
     * -> MQTT gửi cho robot.
     */

    hardware_set_status_callback(
        publish_status
    );


    esp_mqtt_client_config_t mqtt_cfg =
    {
        .broker.address.uri =
            MQTT_BROKER_URI,
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


    ESP_LOGI(
        TAG,
        "MQTT client started"
    );
}