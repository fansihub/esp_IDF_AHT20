/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "unity.h"
#include "driver/i2c.h"
#include "aht20.h"
#include "esp_system.h"
#include "esp_log.h"

static const char *TAG = "aht20 test";

#define TEST_MEMORY_LEAK_THRESHOLD (-400)

#define I2C_MASTER_SCL_IO 5       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 4       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static aht20_dev_handle_t aht20 = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

static void i2c_sensor_ath20_init(void)
{
    aht20_i2c_config_t i2c_conf = {
        .i2c_port = I2C_MASTER_NUM,
        .i2c_addr = AHT20_ADDRRES_0,
    };

    i2c_bus_init();
    aht20_new_sensor(&i2c_conf, &aht20);
    TEST_ASSERT_NOT_NULL_MESSAGE(aht20, "AHT20 create returned NULL");
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    /*     for (int i = 10; i >= 0; i--) {
            printf("Restarting in %d seconds...\n", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("Restarting now.\n");
        fflush(stdout);
        esp_restart(); */
    esp_err_t ret = ESP_OK;
    uint32_t temperature_raw;
    uint32_t humidity_raw;
    float temperature;
    float humidity;

    i2c_sensor_ath20_init();

    while (1)
    {
        TEST_ASSERT(ESP_OK == aht20_read_temperature_humidity(aht20, &temperature_raw, &temperature, &humidity_raw, &humidity));
        ESP_LOGI(TAG, "%-20s: %2.2f %%", "humidity is", humidity);
        ESP_LOGI(TAG, "%-20s: %2.2f degC", "temperature is", temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // aht20_del_sensor(aht20);
    // ret = i2c_driver_delete(I2C_MASTER_NUM);
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
}
