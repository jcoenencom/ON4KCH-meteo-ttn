#include <RadioLib.h>


#include "config.h"


extern SX1276 radio;

extern Stream *console;
extern ESPTelnetStream telnet;
extern unsigned long previousMillis;
extern unsigned long currentMillis;
extern CayenneLPP Payload;

AHT20 aht;
Adafruit_BMP280 bmp;

extern TimerEvent timermeasure;
extern WiFiClient net;
extern MQTTClient client;
extern TimerEvent TTNuplink;


int AnalogPin = A0;

extern volatile int count;   //count magnet detection

extern float speed;



// ============================================================================
// Below is to support the sketch - only make changes if the notes say so ...

// copy over the EUI's & keys in to the something that will not compile if incorrectly formatted
uint64_t joinEUI =   RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI  =   RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

extern LoRaWANNode node;

// create the LoRaWAN node




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
//  digitalWrite(ledPin,);
  

  console = &Serial;
  console->println("Connecting WiFi !");
  
  beginWiFi();
  console->println("WiFi is connected !");
  console->println(WiFi.localIP().toString());
  setupTelnet();
  initOTAserver();

  Wire.begin(D4,D3);
  scanDevices(&Wire);
  if (aht.begin() == false) {
    console->println("AHT20 not detected");
  } else {
   console->println("Found AHT20");
  }
  bmp.begin(0x77);

  console->println(F("Initialise the radio"));
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

  // Setup the OTAA session information
  state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  debug(state != RADIOLIB_ERR_NONE, F("Initialise node failed"), state, true);

  Serial.println(F("Join ('login') the LoRaWAN Network"));
  state = node.activateOTAA();
  debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, true);

  Serial.println(F("Ready!\n"));

  setwind();

}

void loop() {

  ElegantOTA.loop();
  telnet.loop();
  client.loop();
  checktimers();
  //timermeasure.update(); //start timer that calculates the Speed
  //TTNuplink.update(); //start the ttn uplink timer
}
