#include "storage.h"

#include "nvs_flash.h"
#include "nvs.h"

void storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if(ret == ESP_ERR_NVS_NO_FREE_PAGES ||
       ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

void storage_save_floor(int floor)
{
    nvs_handle_t handle;

    if(nvs_open("elevator", NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_i32(handle, "floor", floor);

        nvs_commit(handle);

        nvs_close(handle);
    }
}

int storage_load_floor(void)
{
    nvs_handle_t handle;

    int32_t floor = 1;

    if(nvs_open("elevator", NVS_READONLY, &handle) == ESP_OK)
    {
        nvs_get_i32(handle, "floor", &floor);

        nvs_close(handle);
    }

    return floor;
}