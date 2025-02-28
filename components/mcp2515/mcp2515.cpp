#include "mcp2515.h"

#define MCP_RESET 0xC0
#define MCP_READ 0x03
#define MCP_WRITE 0x02
#define MCP_RTS_TX0 0x81
#define MCP_CANCTRL 0x0F
#define MCP_NORMAL_MODE 0x00
#define MCP_TXB0CTRL 0x30

MCP2515::MCP2515(spi_device_handle_t spi_handle, gpio_num_t cs_pin)
{
    this->spi = spi_handle;
    this->cs = cs_pin;

    gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    gpio_set_level(cs, 1);
}

void MCP2515::startSPI()
{
    gpio_set_level(cs, 0);
}

void MCP2515::endSPI()
{
    gpio_set_level(cs, 1);
}

uint8_t MCP2515::spiTransfer(uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    t.rx_buffer = &data;
    spi_device_transmit(spi, &t);
    return data;
}

void MCP2515::writeRegister(uint8_t reg, uint8_t value)
{
    startSPI();
    spiTransfer(MCP_WRITE);
    spiTransfer(reg);
    spiTransfer(value);
    endSPI();
}

uint8_t MCP2515::readRegister(uint8_t reg)
{
    startSPI();
    spiTransfer(MCP_READ);
    spiTransfer(reg);
    uint8_t result = spiTransfer(0x00);
    endSPI();
    return result;
}

MCP2515::ERROR MCP2515::reset()
{
    startSPI();
    spiTransfer(MCP_RESET);
    endSPI();
    vTaskDelay(pdMS_TO_TICKS(10));
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setNormalMode()
{
    writeRegister(MCP_CANCTRL, MCP_NORMAL_MODE);
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::readMessage(uint8_t *data, uint8_t *length)
{
    *length = 8; // Exemplo: sempre lê 8 bytes
    for (uint8_t i = 0; i < *length; i++)
    {
        data[i] = readRegister(0x66 + i); // Endereço fictício do buffer
    }
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const uint8_t *data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        writeRegister(0x36 + i, data[i]); // Endereço fictício do buffer
    }
    startSPI();
    spiTransfer(MCP_RTS_TX0);
    endSPI();
    return ERROR_OK;
}