# ESP32 CAN-to-Bluetooth Adapter

This project allows an ESP32 to act as a CAN-to-Bluetooth adapter, enabling wireless communication with a vehicle's CAN bus using a Bluetooth connection. The ESP32 interfaces with an MCP2515 CAN controller, which is connected to the vehicle's OBD2 port.

## Features

- Reads CAN bus data from the vehicle via the MCP2515 module.
- Sends CAN messages over Bluetooth using the Serial Port Profile (SPP).
- Compatible with standard CAN analysis tools like `candump` from `can-utils`.
- Runs on ESP32 with ESP-IDF framework.

## Components Required

- ESP32 Development Board
- MCP2515 CAN Module with SPI Interface
- OBD2 to DB9 Adapter (or custom wiring to OBD2 connector)

## Wiring Diagrams

### ESP32 to MCP2515 Connection

```
ESP32         MCP2515
----------------------
3.3V  -----> VCC
GND   -----> GND
GPIO23 ----> MOSI
GPIO19 ----> MISO
GPIO18 ----> SCK
GPIO5  ----> CS
GPIO4  ----> INT
```

### MCP2515 to OBD2 Connection

```
MCP2515      OBD2 Connector
---------------------------
CANH  -----> Pin 6 (CAN High)
CANL  -----> Pin 14 (CAN Low)
GND   -----> Pin 4 (Chassis Ground) // optional
VCC   -----> Pin 16 (Battery Power) // optional
```

## Installation and Setup

### 1. Install ESP-IDF

Ensure ESP-IDF is installed and configured:

```sh
cd ~
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 2. Clone the Project

```sh
git clone https://github.com/leandrobalta/CAN2BT.git
cd CAN2BT
```

### 3. Configure the Project

```sh
idf.py menuconfig
```

Enable:

- **Component Config → Bluetooth → Bluetooth Classic**
- **Component Config → Bluetooth → Enable SPP**

### 4. Build and Flash

```sh
idf.py build
idf.py flash
```

### 5. Monitor Output

```sh
idf.py monitor
```

## Usage

1. Pair with the ESP32 using Bluetooth.
2. Connect to the Bluetooth SPP service.
3. Use `candump` to read CAN messages:

```sh
candump -L can0
```

## Troubleshooting

- Ensure the ESP32 and MCP2515 are properly wired.
- Check the baud rate configuration of the CAN bus.
- Verify Bluetooth pairing and SPP connection.
- Use `idf.py monitor` to check for errors.

## License

This project is open-source and distributed under the MIT License.

