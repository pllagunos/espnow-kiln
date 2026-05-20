# ESPNOW KILN CONTROL SYSTEM

Custom ESP32 gas kiln control system using ESP NOW network for communication with peripherals. 

-- Controller (esp #1): includes TFT display, uses MAX31856 for temperature reading, SD card for storing programs,  SN74hc595 shift register for GPIO expansion for controlling gas valves and relays (i.e for blower motor).
-- espnow slaves (esp #2 and esp #3): esp32 devices that read differencial pressure transducers, control actuators and communicate with ESP32 control system, ESP #3 uploads telemetry data (from controller and dpt nodes) to influxdb cloud and shows status in a small OLED screen.
