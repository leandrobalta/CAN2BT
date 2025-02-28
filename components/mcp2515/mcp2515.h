#ifndef MCP2515_H
#define MCP2515_H



#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>

#define MCP_ANY 0
#define CAN_500KBPS 500000
#define MCP_8MHZ 8000000
#define MCP2515_OK 0

class MCP2515 {
public:
    MCP2515(spi_device_handle_t spi, gpio_num_t cs);
    
    enum ERROR {
        ERROR_OK = 0,
        ERROR_FAIL = 1
    };

    ERROR reset(void);
    ERROR setNormalMode();
    ERROR readMessage(uint8_t *data, uint8_t *length);
    ERROR sendMessage(const uint8_t *data, uint8_t length);

private:
    spi_device_handle_t spi;
    gpio_num_t cs;

    void startSPI();
    void endSPI();
    uint8_t spiTransfer(uint8_t data);
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

#endif // MCP2515_H
