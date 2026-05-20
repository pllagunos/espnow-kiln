# Hardware

## Controller

This project was built on a perfboard, so no PCB gerber file is available.

`Sheet_1.png` shows how the ESP32 controller is wired. A custom JST connector is used to connect to the TFT display. The buttons for the TFT display are placed in a perfboard and connect via another JST connector per `Sheet_2.png`.

A much better hardware design can be found in my other repositories:

* [Modbus kiln controller](https://github.com/pllagunos/ModbusKiln)
* [Electric kiln controller](https://github.com/pllagunos/esp32-kiln-controller)

## ESPNOW nodes

See `espnow-nodes` for easyeda files and fabrication file (Gerber).

![espnow](/hardware/espnow-nodes/image.png)