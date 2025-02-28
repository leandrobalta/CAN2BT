# CAN Bus-to-Bluetooth Adapter for ESP32

## Description
This project is an open-source **CAN Bus-to-Bluetooth** adapter using an **ESP32** and **MCP2515**. It allows you to wirelessly connect to a CAN network using **Bluetooth Serial (SPP)** and is designed to be compatible with **SocketCAN**, **python-can**, and other tools supporting the **slcan (Serial CAN)** protocol.

This adapter functions similarly to a CAN-to-USB interface but transmits data over Bluetooth, enabling wireless diagnostics, logging, and interaction with automotive and industrial CAN networks.

## Features
- **MCP2515-based CAN Bus interface** via SPI
- **Bluetooth Serial (SPP)** for wireless connection
- **slcan protocol** for compatibility with standard CAN tools
- **ESP-IDF-based firmware** for robust performance
- **Fully open-source and DIY-friendly**

## Hardware Requirements
- **ESP32** (any model with Bluetooth support)
- **MCP2515 CAN Bus Module**
- **CAN Bus network (vehicle, industrial, etc.)**
- **3.3V Logic Level Converter (if required for MCP2515 module)**

## Wiring Diagram
| MCP2515 | ESP32 |
|----------|------|
| VCC (3.3V) | 3.3V |
| GND | GND |
| CS (Chip Select) | GPIO5 |
| SCK (Clock) | GPIO18 |
| MOSI (Master Out) | GPIO23 |
| MISO (Master In) | GPIO19 |
| INT (Interrupt) | GPIO4 |
| TXCAN | CANH |
| RXCAN | CANL |

## Installation
### 1. Setup ESP-IDF Environment
Ensure you have the **ESP-IDF** development environment set up on your system. If not, follow the [official guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

### 2. Clone the Repository
```sh
git clone https://github.com/leandrobalta/CAN2BT.git
cd CAN2BT
```

### 3. Build and Flash Firmware
```sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
Replace `/dev/ttyUSB0` with your ESP32's actual port.

## Usage
### Connect via Bluetooth
1. Pair your **PC/Mac/Linux** with the ESP32 via Bluetooth settings.
2. Find the ESP32 Bluetooth address:
   ```sh
   hcitool scan
   ```
3. Connect to the Bluetooth Serial Port:
   ```sh
   rfcomm connect hci0 <ESP32_MAC_ADDRESS>
   ```

### Setup CAN Interface (Linux)
```sh
sudo slcand -o -c -s6 /dev/rfcomm0 can0
sudo ifconfig can0 up
candump can0
```

Now, you can receive and send CAN messages wirelessly!

## License
This project is licensed under the **MIT License**. Feel free to modify and contribute!

---
**Contributions Welcome!** If you find issues or have improvements, feel free to open a Pull Request. 🚀

