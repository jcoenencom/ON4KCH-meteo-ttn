ESP8266 controlled Weather station using mqtt local broker and TheThingNetwork.

Used module and GPIO pins used.

Module AHT20/BMP280 I2C (D4 SCL, D3 SCA)
Module LoRa RFM95W  DI00, 3,3 Volts|, GND, MISO, MOSI, SCK, NSS.
Module Hal KY-024  D02 (nécéssite une résistance pull up et doit pouvoir générer une interrupt).

optional an OLED display Module, can be added on the I2C bus (not yet foreseen in the code as it does not make a lot of sense).
