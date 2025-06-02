ESP8266 controlled Weather station using mqtt local broker and TheThingNetwork.

Used module and GPIO pins used.

Module AHT20/BMP280 I2C (D4 SCL, D3 SCA)
Module LoRa RFM95W  DI00, 3,3 Volts|, GND, MISO, MOSI, SCK, NSS.
Module Hal KY-024  D02 (nécéssite une résistance pull up et doit pouvoir générer une interrupt).

optional an OLED display Module, can be added on the I2C bus (not yet foreseen in the code as it does not make a lot of sense).

![image](https://github.com/user-attachments/assets/7711f28d-f23a-42e3-85b0-e01ea34d78ea)



Code is functional, data is collected from the broker into a RRD database for wuick plotting and long term statistics.
Providing the broker has a websocket access, data can be fed dynamically in web pages (using a small javascript script).

The weather station enclosure come from 
