#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "../protocol/packet.h"

void elevator_process_request(
    const RobotRequest *request,
    ElevatorResponse *response);

#endif