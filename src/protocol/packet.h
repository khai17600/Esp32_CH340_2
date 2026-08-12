#ifndef PACKET_H
#define PACKET_H

#define MAX_ID_LEN       16
#define MAX_COMMAND_LEN  32
#define MAX_STATUS_LEN   32
#define MAX_STATE_LEN    32

typedef struct
{
    char robot_id[MAX_ID_LEN];

    char command[MAX_COMMAND_LEN];

    int source_floor;

    int destination_floor;

    int request_id;

} RobotRequest;

typedef struct
{
    int request_id;

    char status[MAX_STATUS_LEN];

    char elevator_state[MAX_STATE_LEN];

    int current_floor;

} ElevatorResponse;

#endif