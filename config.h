//#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "ESPTelnetStream.h"
#include <MQTT.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <AHT20.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>

#include <CayenneLPP.h>


#include <TimerEvent.h>



#include <RadioLib.h>
#define MAX_SIZE 200 //TTN message max size
#define UPLINKPERIOD 10UL*60UL*1000UL
#define DIO1_PIN D8 // NSS
#define DIO0_PIN D1 // DIO0
#define RESET_PIN 14 // RESET
#define SCK_PIN D5 // SCK for SPI1
#define MOSI_PIN D7 // MOSI for SPI1
#define MISO_PIN D6  // MISO for SPI1
#define CS_PIN D8    // CS for SPI1





#define RADIOLIB_LORAWAN_JOIN_EUI  0x0000000000000000

// the Device EUI & two keys can be generated on the TTN console 
#ifndef RADIOLIB_LORAWAN_DEV_EUI   // Replace with your Device EUI
#define RADIOLIB_LORAWAN_DEV_EUI   0x70B3D57ED0070D18
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY   // Replace with your App Key 
#define RADIOLIB_LORAWAN_APP_KEY   0x2C, 0x60, 0x27, 0x36, 0x6A, 0x3A, 0xB2, 0xB4, 0x61, 0x10, 0xDC, 0x26, 0xB6, 0x57, 0x88, 0x7B
#endif
#ifndef RADIOLIB_LORAWAN_NWK_KEY   // Put your Nwk Key here
#define RADIOLIB_LORAWAN_NWK_KEY   0x1B, 0x7C, 0x77, 0x81, 0x62, 0x9E, 0xAC, 0xFB, 0xB4, 0x5F, 0xCA, 0x95, 0x32, 0x09, 0x89, 0x6E
#endif



#ifndef FIXEDIF
  #define FIXEDIP
  #define STATICIP (192,168,1,79)  // static IP address
  #define SUBNET (255,255,255,0)
  #define GATEWAY (192,168,1,1)
  #define PRIMARYDNS (192,168,1,31)  // optional
  #define SECONDARYDNS (192,168,1,1) // optional
#endif //FIXEDIP

#define HOSTNAME "on4kch_meteo"
#define BROKER "raspdomo.local"


#define TOPICSTATE "on4kch/meteo/state"
#define TOPIC "on4kch/meteo/speed"
#define TOPICSPD "on4kch/meteo/spd"

#define SPEEDTIMER 5000UL  //accumulate hit for 5 seconds
#define RADIUS 0.028 // 28 mm de rayon
#define MQTTTIMER 30000UL //mqtt upload timer 10 seconds
#define DEBOUNCINGTIME 50UL;

typedef struct {
    float temperature;
    float temperature2;
    float pressure;
    int humidity;
}
measure;

typedef struct {
  unsigned long sinterval;
  unsigned long mqtt;
  unsigned long ttn;
} tlimits;


void TTNsend();
void checktimers();
void beginWiFi();
void setupTelnet();
void initOTAserver();
void scanDevices(TwoWire *w);
measure thp();
char* prtmes();
void setwind();
void IRAM_ATTR counter();
void initSensors();
String stateDecode(const int16_t result);
void debug(bool failed, const __FlashStringHelper* message, int state, bool halt);
void arrayDump(uint8_t *buffer, uint16_t len);
void Speed();

void connect();
void reconnect();
