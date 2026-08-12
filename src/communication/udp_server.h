#ifndef UDP_SERVER_H
#define UDP_SERVER_H

typedef void (*udp_receive_callback_t)(
    const char *data,
    int len,
    const char *robot_ip,
    int robot_port);

void udp_server_start(
    udp_receive_callback_t callback);

void udp_server_send(
    const char *ip,
    int port,
    const char *data);

#endif