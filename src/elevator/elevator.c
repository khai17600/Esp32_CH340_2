#include <stdio.h>
#include <string.h>

#include "elevator.h"

void elevator_process_request(
    const RobotRequest *request,
    ElevatorResponse *response)
{
    printf("\n");
    printf("===== Elevator Controller =====\n");

    /*
     * Hiện tại chỉ mô phỏng.
     * Sau này chỗ này sẽ điều khiển PLC,
     * Relay hoặc Modbus.
     */

    printf("Robot %s requested elevator\n",
           request->robot_id);

    printf("From floor %d -> %d\n",
           request->source_floor,
           request->destination_floor);

    /*
     * Sinh response
     */

    response->request_id = request->request_id;

    strncpy(response->status,
            "ACCEPTED",
            MAX_STATUS_LEN - 1);
    response->status[MAX_STATUS_LEN - 1] = '\0';

    strncpy(response->elevator_state,
            "MOVING",
            MAX_STATE_LEN - 1);
    response->elevator_state[MAX_STATE_LEN - 1] = '\0';

    response->current_floor =
            request->source_floor;

    printf("Request accepted.\n");

    printf("===============================\n");
}