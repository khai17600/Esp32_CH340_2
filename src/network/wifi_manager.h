#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

void wifi_init(void);

/*
 * Block cho tới khi WiFi có IP.
 * Trả về true nếu kết nối thành công trong thời gian timeout_ms,
 * false nếu hết thời gian mà vẫn chưa có IP.
 */
bool wifi_wait_connected(uint32_t timeout_ms);

#endif