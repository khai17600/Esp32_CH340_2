#include "communication/udp_server.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "esp_log.h"

#define UDP_PORT 5000
#define BUFFER_SIZE 1024

static const char *TAG = "UDP";

static udp_receive_callback_t receive_callback = NULL;

static void udp_server_task(void *arg)
{
    char rx_buffer[BUFFER_SIZE];

    struct sockaddr_in server_addr;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (sock < 0)
    {
        ESP_LOGE(TAG, "Cannot create socket");
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock,
                   (struct sockaddr *)&server_addr,
                   sizeof(server_addr));

    if (err < 0)
    {
        ESP_LOGE(TAG, "Bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP Server started");
    ESP_LOGI(TAG, "Listening on port %d", UDP_PORT);

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);

        int len = recvfrom(sock,
                           rx_buffer,
                           sizeof(rx_buffer) - 1,
                           0,
                           (struct sockaddr *)&source_addr,
                           &socklen);

        if (len < 0)
            continue;

        rx_buffer[len] = '\0';

        char robot_ip[16];

        inet_ntop(AF_INET,
                  &source_addr.sin_addr,
                  robot_ip,
                  sizeof(robot_ip));

        int robot_port = ntohs(source_addr.sin_port);

        ESP_LOGI(TAG,
                 "Packet from %s:%d",
                 robot_ip,
                 robot_port);

        if (receive_callback != NULL)
        {
            receive_callback(
                rx_buffer,
                len,
                robot_ip,
                robot_port);
        }
    }
}

void udp_server_start(udp_receive_callback_t callback)
{
    receive_callback = callback;

    xTaskCreate(
        udp_server_task,
        "udp_server",
        4096,
        NULL,
        5,
        NULL);
}

void udp_server_send(const char *ip,
                     int port,
                     const char *data)
{
    struct sockaddr_in dest_addr;

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr(ip);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (sock < 0)
    {
        ESP_LOGE(TAG, "Cannot create socket");
        return;
    }

    sendto(sock,
           data,
           strlen(data),
           0,
           (struct sockaddr *)&dest_addr,
           sizeof(dest_addr));

    close(sock);

    ESP_LOGI(TAG, "Reply sent");
}