#include <stdio.h>
#include "driver/spi_master.h" // NOVO: Biblioteca do SPI do ESP-IDF
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "mcp2515.h" // Biblioteca MCP2515

#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CLK 18
#define SPI_CS 5
#define CAN_INT 4

#define SPP_SERVER_NAME "ESP32_CAN_BT"

static const char *TAG = "CAN_BLUETOOTH";

static uint32_t spp_client = 0;

void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "SPP Connection Opened");
        spp_client = param->open.handle;
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP Connection Closed");
        spp_client = 0;
        break;

    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(TAG, "Received %d bytes from Bluetooth", param->data_ind.len);
        // maybe process the data received and send to car canbus
        break;

    default:
        break;
    }
}

void setup_bluetooth()
{
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "Bluetooth release BLE memory failed %s", esp_err_to_name(ret));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "Bluetooth controller initialize failed %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret)
    {
        ESP_LOGE(TAG, "Bluetooth controller enable failed %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "Bluedroid initialize failed %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "Bluedroid enable failed %s", esp_err_to_name(ret));
        return;
    }

    esp_spp_register_callback(spp_callback);
    esp_spp_init(ESP_SPP_MODE_CB);
    esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);

    ESP_LOGI(TAG, "Bluetooth SPP initialized");
}

void app_main()
{
    ESP_LOGI(TAG, "Starting CAN Bluetooth Adapter...");

    setup_bluetooth();

    // NOVO: Configuração do SPI para ESP-IDF
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0, // Modo SPI (0, 1, 2, 3)
        .duty_cycle_pos = 0,
        .cs_ena_posttrans = 0,
        .cs_ena_pretrans = 0,
        .clock_speed_hz = 1000000, // 1MHz
        .spics_io_num = SPI_CS,
        .queue_size = 1,
    };
    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    // NOVO: Inicialização do MCP2515 usando ESP-IDF
    mcp2515 mcp(spi, GPIO_NUM_5);
    if (mcp.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == MCP2515_OK)
    {
        ESP_LOGI(TAG, "MCP2515 initialized successfully!");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize MCP2515");
        return;
    }

    while (1)
    {
        struct can_frame frame;
        if (mcp.readMessage(&frame) == MCP2515_OK)
        {
            ESP_LOGI(TAG, "CAN Message received: ID=0x%X, Data", frame.can_id);
            for (int i = 0; i < frame.can_dlc; i++)
            {
                printf("%02X ", frame.data[i]);
            }
            printf("\n");

            if (spp_client)
            {
                esp_spp_write(spp_client, frame.data, frame.can_dlc);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}