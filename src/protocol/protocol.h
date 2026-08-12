#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "packet.h"

/*
 * Xử lý một yêu cầu từ Robot (đã parse thành struct)
 */
void protocol_process_request(
    const RobotRequest *request,
    ElevatorResponse *response);

/*
 * Xử lý raw packet nhận từ UDP: parse JSON,
 * gọi protocol_process_request, gửi response ngược lại
 */
void protocol_process_packet(
    const char *data,
    int len,
    const char *robot_ip,
    int robot_port);

#endif