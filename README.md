# esp32 i2c slave example

this example file implements a esp32 as slave that can communicate with another esp32 (as master)

## HW setup

ESP32 master:
* SDA: GPIO 18
* SCL: GPIO 19
* GND

wired to ESP32 slave:
* SDA: GPIO 21
* SCL: GPIO 22
* GND

NOTE: it turned out to be very important to connect GND of both esp32 (just connecting them via USB to the same Computer is not enough!)

Find the code for the master [here](https://github.com/juri117/esp32_i2c_master).