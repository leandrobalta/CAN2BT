#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "mcp2515.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"

#define TAG "CAN_BLUETOOTH"
#define SPP_SERVER_NAME "ESP32_CAN_BT"

// SPI Pins
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CLK 18
#define SPI_CS 5
#define CAN_INT 4

static uint32_t spp_client = 0;
static MCP2515_t mcp;

/**
 * Callback for Bluetooth SPP events
 */
void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "SPP connection opened");
        spp_client = param->open.handle;
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP connection closed");
        spp_client = 0;
        break;

    default:
        break;
    }
}

/**
 * Bluetooth Classic (SPP) setup
 */
void setup_bluetooth()
{
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to release BLE memory: %s", esp_err_to_name(ret));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable Bluetooth Classic: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    esp_spp_register_callback(spp_callback);
    esp_spp_enhanced_init(ESP_SPP_MODE_CB);
    esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);

    ESP_LOGI(TAG, "Bluetooth SPP started");
}

/**
 * MCP2515 setup
 */
void setup_can()
{
    // Configure SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz
        .mode = 0,
        .spics_io_num = SPI_CS,
        .queue_size = 1,
    };

    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    // Initialize MCP2515
    mcp->spi = spi;
    if (MCP2515_init(&mcp) == ESP_OK)
    {
        ESP_LOGI(TAG, "MCP2515 successfully initialized!");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize MCP2515!");
        return;
    }

    MCP2515_setBitrate(CAN_500KBPS, MCP_8MHZ);
    MCP2515_setNormalMode(&mcp);
}

/**
 * Main function
 */
void app_main()
{
    ESP_LOGI(TAG, "Starting ESP32 CAN-to-Bluetooth Adapter");

    setup_bluetooth();
    setup_can();

    struct can_frame frame;
    while (1)
    {
        if (MCP2515_readMessage(RXB0, &frame) == ESP_OK)
        {
            ESP_LOGI(TAG, "CAN message received: ID=0x%X, DLC=%d", frame.can_id, frame.can_dlc);

            // Send via Bluetooth
            if (spp_client)
            {
                esp_spp_write(spp_client, frame.can_dlc, frame.data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
