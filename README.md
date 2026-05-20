# ESP-NOW Kiln Control System

ESP32-based gas kiln controller built around an ESP-NOW network.

The project is split into three nodes:

- Controller (`ESP32 #1`): the main kiln controller with TFT UI, MAX31856 thermocouple input, SD-backed firing programs, PID control, and SN74HC595 GPIO expansion for valves and relays.
- DPT node (`ESP32 #2`): reads differential pressure transducers, drives the chimney output, and exchanges data over the ESP-NOW network.
- Telemetry node (`ESP32 #3`): reads pressure data, shows local status on an OLED, receives kiln state from the network, and publishes telemetry to InfluxDB.

## Repository Layout

- `controller/`: PlatformIO project for the main controller.
- `controller/src/main.ino`: kiln UI, program execution, PID control, and ESP-NOW coordination.
- `dpt-node/dpt-node.ino`: pressure sensor and chimney actuator node.
- `telemetry-node/ESP_NOW_3.ino`: telemetry and OLED status node.

## Hardware Overview

The controller firmware is written for an ESP32 DevKit using:

- MAX31856 for thermocouple measurement.
- ILI9341 display via `TFT_eSPI`.
- SN74HC595 shift register for output expansion.
- SD card storage for firing programs.
- ESP-NOW for low-latency communication between nodes.

The peripheral nodes are also ESP32-based and use analog inputs for pressure transducers. The telemetry node additionally uses Wi-Fi for cloud upload and a small OLED for status display.

## Software Stack

- PlatformIO + Arduino framework for the controller.
- Arduino-style sketches for the peripheral nodes.
- `TFT_eSPI`, `PID`, and `Adafruit MAX31856` on the controller.
- `InfluxDbClient` and `SSD1306Wire` on the telemetry node.

Controller build environment:

- Board: `esp32doit-devkit-v1`
- Framework: `arduino`
- Monitor speed: `115200`

## Getting Started

### 1. Build the controller

The main controller is a PlatformIO project:

```bash
cd controller
pio run
```

To flash and monitor:

```bash
cd controller
pio run --target upload
pio device monitor
```

### 2. Flash the peripheral nodes

The node sketches live in:

- `dpt-node/dpt-node.ino`
- `telemetry-node/ESP_NOW_3.ino`

Open and flash them with your preferred Arduino-compatible ESP32 workflow.

## Configuration

Before deploying to hardware, update the node-specific settings in source:

- Wi-Fi SSID and password placeholders.
- ESP-NOW peer MAC addresses.
- InfluxDB URL, token, organization, and bucket.
- Sensor calibration constants and pin assignments.
- PID tunings and kiln-specific timing limits.

The current repository is set up with placeholder values for credentials, but hardware-specific identifiers and calibration values should still be reviewed before publishing or deploying.

## Notes

- This project controls combustion-related hardware. Review all outputs, fail-safe behavior, and temperature limits before running it on a live kiln.

## License

This repository is available under the MIT License. See `LICENSE` for details.
