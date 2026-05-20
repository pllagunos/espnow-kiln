### Horno Gas (Gas Kiln)

Custom ESP32 gas kiln control system using ESP NOW network for communication with peripherals. Includes TFT display, uses MAX31856 for temperature reading, SD card for storing programs,  SN74hc595 shift register for GPIO expansion for controlling gas valves and motors.
-- espnow slaves (1,2): esp32 devices that read differencial pressure transducers, control actuators and communicate with ESP32 control system, slave #2 also uploads telemetry data (from HornoGas and pressures) to influxdb cloud and shows status in a small OLED screen.

** change tft lib to be included in platformio.ini file