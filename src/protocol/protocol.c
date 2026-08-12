#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "cJSON.h"

#include "protocol.h"
#include "../elevator/elevator.h"
#include "../communication/udp_server.h"

static const char *TAG = "PROTOCOL";

void protocol_process_request(
    const RobotRequest *request,
    ElevatorResponse *response)
{
    printf("\n");
    printf("========== Robot Request ==========\n");

    printf("Robot ID          : %s\n", request->robot_id);
    printf("Command           : %s\n", request->command);
    printf("Source Floor      : %d\n", request->source_floor);
    printf("Destination Floor : %d\n", request->destination_floor);
    printf("Request ID        : %d\n", request->request_id);

    printf("===================================\n");

    /*
     * Gọi bộ điều khiển thang máy
     */
    elevator_process_request(request, response);
}

void protocol_process_packet(
    const char *data,
    int len,
    const char *robot_ip,
    int robot_port)
{
    cJSON *json = cJSON_ParseWithLength(data, len);
    if (json == NULL)
    {
        ESP_LOGE(TAG, "Invalid JSON");
        return;
    }

    RobotRequest request = {0};

    const cJSON *robot_id = cJSON_GetObjectItem(json, "robot_id");
    const cJSON *command   = cJSON_GetObjectItem(json, "command");
    const cJSON *src        = cJSON_GetObjectItem(json, "source_floor");
    const cJSON *dst        = cJSON_GetObjectItem(json, "destination_floor");
    const cJSON *req_id     = cJSON_GetObjectItem(json, "request_id");

    if (!cJSON_IsString(robot_id) || !cJSON_IsString(command) ||
        !cJSON_IsNumber(src) || !cJSON_IsNumber(dst) || !cJSON_IsNumber(req_id))
    {
        ESP_LOGE(TAG, "Missing/invalid fields");
        cJSON_Delete(json);
        return;
    }

    strncpy(request.robot_id, robot_id->valuestring, MAX_ID_LEN - 1);
    request.robot_id[MAX_ID_LEN - 1] = '\0';

    strncpy(request.command, command->valuestring, MAX_COMMAND_LEN - 1);
    request.command[MAX_COMMAND_LEN - 1] = '\0';

    request.source_floor      = src->valueint;
    request.destination_floor = dst->valueint;
    request.request_id        = req_id->valueint;

    cJSON_Delete(json);

    ElevatorResponse response = {0};
    protocol_process_request(&request, &response);

    /*
     * Serialize response và gửi ngược lại robot
     */
    cJSON *resp_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(resp_json, "request_id", response.request_id);
    cJSON_AddStringToObject(resp_json, "status", response.status);
    cJSON_AddStringToObject(resp_json, "elevator_state", response.elevator_state);
    cJSON_AddNumberToObject(resp_json, "current_floor", response.current_floor);

    char *resp_str = cJSON_PrintUnformatted(resp_json);

    udp_server_send(robot_ip, robot_port, resp_str);

    cJSON_free(resp_str);
    cJSON_Delete(resp_json);
}