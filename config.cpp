#include "config.h"

int wstatus = WL_IDLE_STATUS;                     // the Wifi radio's status

IPAddress ip STATICIP;
IPAddress gateway GATEWAY;
IPAddress subnet SUBNET;
IPAddress primaryDNS PRIMARYDNS;
IPAddress secondaryDNS SECONDARYDNS;
uint16_t  port = 23;

char ssid[] = "Malperthuis";
char pass[] = "9725145239910203";


WiFiClient net;
MQTTClient client;





Stream *console;
ESPTelnetStream telnet;
String telin="";
String lastcmd="help";
char uparrow[] = {'\x1B', '\x5B', '\x41'};


const uint32_t uplinkIntervalSeconds = 5UL * 60UL;    // minutes x seconds
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
unsigned long uplinkmillis = 0;
uint32_t minimumDelay = uplinkIntervalSeconds * 1000UL;
uint32_t interval = 1000UL;  //node.timeUntilUplink();     // calculate minimum duty cycle delay (per FUP & law!)
uint32_t delayMs = max(interval, minimumDelay); // cannot send faster than duty cycle allows
extern int buttonState;
uint32_t deviceOnline = 0x00;

float speed;
float RPM;
volatile int count = 0;   //count magnet detection must be volatile as it is read bey Speed outside of intr routine


char* meastxt;

extern AHT20 aht;
extern Adafruit_BMP280 bmp;

int16_t state = 0;  // return value for calls to RadioLib


const unsigned int measureperiod = SPEEDTIMER; // periode pour prendre une mesure de tour effectués
TimerEvent timermeasure;

const unsigned int ttnuplinkperiod=UPLINKPERIOD;

TimerEvent TTNuplink;



SX1276 radio = new Module(CS_PIN, DIO0_PIN, RADIOLIB_NC, RADIOLIB_NC);

enum {
    POWERMANAGE_ONLINE  = _BV(0),
    DISPLAY_ONLINE      = _BV(1),
    RADIO_ONLINE        = _BV(2),
    GPS_ONLINE          = _BV(3),
    PSRAM_ONLINE        = _BV(4),
    SDCARD_ONLINE       = _BV(5),
    AXDL345_ONLINE      = _BV(6),
    BME280_ONLINE       = _BV(7),
    BMP280_ONLINE       = _BV(8),
    BME680_ONLINE       = _BV(9),
    QMC6310_ONLINE      = _BV(10),
    QMI8658_ONLINE      = _BV(11),
    PCF8563_ONLINE      = _BV(12),
    OSC32768_ONLINE      = _BV(13),
};

CayenneLPP Payload(MAX_SIZE);

//delay(delayMs);

const LoRaWANBand_t Region = EU868;

// subband choice: for US915/AU915 set to 2, for CN470 set to 1, otherwise leave on 0
const uint8_t subBand = 0;
LoRaWANNode node(&radio, &Region, subBand);

void connect() {

  //connect first time the mqtt 
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  
  while (!client.connect(BROKER, "jcoenen", "Beatr1ce")) {
    Serial.print(".");
    delay(1000);
  }

  client.publish(TOPICSTATE, "starting");  //client.subscribe("/hello");
  // client.unsubscribe("/hello");
}


void reconnect() {
  // reconnect the mqtt 
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.begin(ssid, pass);
    }

    if (!telnet.isConnected()) {
      setupTelnet();
    }

    if (!client.connected()) {
    console->println("conneccting mqtt");
    while (!client.connect(BROKER, "jcoenen", "Beatr1ce")) {
      Serial.print(".");
      delay(1000);
    }
    client.publish(TOPICSTATE, "reconnecting");
  }

}


void beginWiFi() {
  console->println("Connecting WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin("Malperthuis","9725145239910203");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    console->print(".");
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  ip = WiFi.localIP();
  console->print("Local Wifi :");
  console->println(ip);
  client.begin("192.168.1.18", net);
  connect();
//  client.onMessage(messageReceived);
}

AsyncWebServer server(80);


unsigned long ota_progress_millis = 0;
// OTA definition and call back

void onOTAStart() {
  // Log when OTA has started
  console->println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    console->printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    console->println("OTA update finished successfully!");
  } else {
    console->println("There was an error during OTA update!");
  }
}


void initOTAserver() {
  String host = HOSTNAME ;
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
          char buf[256];
          meastxt=prtmes();
          sprintf(buf,"<H1> minid1Test </H1> Hello, TTGO Meteo stations<br>OTA in /update<br>%s",meastxt);
          request->send(200, "text/HTML; charset=utf-8", buf);
      });
  ElegantOTA.begin(&server);    // Start ElegantOTA

    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

  //start web Server
    server.begin();
  
}


void errorMsg(String error, bool restart = true) {
  console->println(error);
  if (restart) {
    console->println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

void prompt() {
  console->print("on4kch-meteo > ");
}


void onTelnetConnect(String ip) {
  console->print("- Telnet: ");
  console->print(ip);
  console->println(" connected");
    
  console->println("\nWelcome " + telnet.getIP());
  console->println("(Use ^] + q  to disconnect.)");
  console->println("redirecting console to telnet");
  console=&telnet;
  console->println("Console redirected.");
  prompt();
}

void onTelnetDisconnect(String ip) {
  console = &Serial;
  console->println("Console redirected");
  console->print("- Telnet: ");
  console->print(ip);
  console->println(" disconnected");
}

void onTelnetReconnect(String ip) {
  console->print("- Telnet: ");
  console->print(ip);
  console->println(" reconnected");
  prompt();
}

void onTelnetConnectionAttempt(String ip) {
  console->print("- Telnet: ");
  console->print(ip);
  console->println(" tried to connected");
}

void onTelnetInput(String str) {
  // checks for a certain command when a line feed is detected
  if ( str[0] == '\n' ) {
    if (telin[0] == '\x1b') {
        for (size_t i = 0; i < telin.length(); i++)
        {
            Serial.printf("%02X ", telin[i]);
        }
      telin == lastcmd;
    }
    if (telin == "ping") {
    //  console->println("> pong"); 
        console->println("- Telnet: pong");
    // disconnect the client
    } else if (telin == "time") {
        char text[128];
        long left = ((long) millis() - (long) previousMillis - (long) measureperiod )/1000;
        long leftup = ((long) millis() - (long) uplinkmillis - (long) delayMs )/1000;
        sprintf(text,"Time until next. measure: %d\tUntil next uplink %d",int(left),int(leftup));
        console->print(text);
        /* console->print("current ms ");
        console->println(currentMillis);

        console->print("previous ms ");
        console->println(previousMillis);

        console->print("Delay ");
        console->println(delayMs);
        */ 
    } else if (telin == "bye") {
        console->println("> disconnecting you...");
        telnet.disconnectClient();
    }  else if (telin == "read") {
      int sensorvalue = analogRead(A0);
      console->printf("KY-024 Analog %d \t",sensorvalue);
      meastxt=prtmes();
    } else if (telin =="scan") {
      scanDevices(&Wire);
    } else if (telin =="send") {
      TTNsend();
    } else if (telin == "timer") {
        console->print("Measure timer: ");
        console->println(measureperiod);
        console->print("uplink timer: ");
        console->println(ttnuplinkperiod);
    } else if (telin == "raz") {
        errorMsg("rebooting");
    } else {
        console->print("unrecognized command "+telin+"\n available commands are:\n");
        console->println("- ping check\n- time until next uplink\n- bye exit telnet\n- raz reboot module\nread read and display sensor values\nscan scan I2C\nsend send TTN payload");
        prompt();
    }
      lastcmd = telin; 
      telin = "";
    prompt();
    } else {
        if (telin.equals(uparrow)) {
          telin = lastcmd;
          str[0] = '\x0d';
        }
        //add characters received if not carriage return (part of end of string from connectinf telnet client)
        if ( str[0] != 0x0d ) {
            telin += str;
        }
    }
  }


void setupTelnet() {  
  // passing on functions for various telnet events
  console->println("Setting up telnet");
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);

  console->print("- Telnet: ");
  if (telnet.begin(port)) {
    console->println("running");
  } else {
    console->println("error.");
    errorMsg("Will reboot...");
  }
}


void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    console->println("I2C Devices scanning");
    for (addr = 1; addr < 127; addr++) {
        start = millis();
        w->beginTransmission(addr); delay(2);
        err = w->endTransmission();
        if (err == 0) {
            nDevices++;
            switch (addr) {
            case 0x77:
            case 0x76:
                console->println("\tFind BMX280 Sensor!");
                deviceOnline |= BME280_ONLINE;
                break;
            case 0x34:
                console->println("\tFind AXP192/AXP2101 PMU!");
                deviceOnline |= POWERMANAGE_ONLINE;
                break;
            case 0x3C:
                console->println("\tFind SSD1306/SH1106 dispaly!");
                deviceOnline |= DISPLAY_ONLINE;
                break;
            case 0x51:
                console->println("\tFind PCF8563 RTC!");
                deviceOnline |= PCF8563_ONLINE;
                break;
            case 0x1C:
                console->println("\tFind QMC6310 MAG Sensor!");
                deviceOnline |= QMC6310_ONLINE;
                break;
            default:
                console->print("\tI2C device found at address 0x");
                if (addr < 16) {
                    console->print("0");
                }
                console->print(addr, HEX);
                console->println(" !");
                break;
            }

        } else if (err == 4) {
            console->print("Unknow error at address 0x");
            if (addr < 16) {
                console->print("0");
            }
            console->println(addr, HEX);
        }
    }
    if (nDevices == 0)
        console->println("No I2C devices found\n");

    console->println("Scan devices done.");
    console->println("\n");
}

measure thp(){
  /* 
  return a structure with measured values from the sensors
  REQUIRES initialization of I2C devices:
    AHT20 aht;
    Adafruit_BMP280 bmp;
  */

  measure retval;
  retval.temperature = aht.getTemperature();
  retval.humidity = aht.getHumidity();
  retval.pressure = bmp.readPressure() / 100.0;
  retval.temperature2=bmp.readTemperature();
  return retval;
}


char* prtmes(){
/* call for a measure and print the result */
  measure val;
  char retval[256];
  val = thp();
  JsonDocument mqttmsg;
  char output[256];
  int local=count;
  sprintf(retval,"counter: %d hits RPM %d (T/min) speed %8.4f (km/hr)\tAHT20 - Temp: %6.2f °C Humidity: %d %%\tBMP280 - Temp: %6.2f °C Pressure: %7.2f hPa\n",local,int(RPM), speed,val.temperature, val.humidity,val.temperature2, val.pressure);
  Payload.reset();
  Payload.addTemperature(0, val.temperature);
  mqttmsg["temperature"] = val.temperature;
  Payload.addTemperature(1,val.temperature2);
  mqttmsg["temperature_2"] = val.temperature2;
  Payload.addRelativeHumidity(2, float(val.humidity));
  mqttmsg["humidity"] = val.humidity;
  Payload.addBarometricPressure(3, val.pressure);
  mqttmsg["pressure"] = val.pressure;
  Payload.addAnalogInput(4, RPM);
  mqttmsg["RPM"] = RPM;
  Payload.addAnalogInput(5, speed);
  mqttmsg["speed"] = speed;
  console->print(retval);
  serializeJson(mqttmsg, output);
  int len = strlen(output)+1;
  reconnect();  // make sure wifi and mqqt are connected if not reconnect
  client.publish(TOPIC, output,len,true); //publish mqtt TOPIC message with retain on
  return retval;
}


/* Timer definition 
struct Timers holding hte next time occurence of timer
  unsigned long uplink; TTN uplink timer
  unsigned long mqtt; mqtt uplink timer
  unsigned long sinterval; speed measurement interval 
*/


tlimits lestimer {0,0,0};
//lestimer.sinterval = millis()+SPEEDTIMER;
//lestimer.mqtt = millis()+MQTTTIMER
//lestimer.ttn = millis()+UPLINKPERIOD;

const byte interruptPin = D2;



void checktimers() {
  unsigned long now=millis();
  if (now >= lestimer.sinterval) {
    lestimer.sinterval = now + SPEEDTIMER; //set timer for next time occurence
    Speed(); // compute the speed
    reconnect();  // make sure wifi and mqqt are connected if not reconnect
    client.publish(TOPICSPD, String(speed)); //publish mqtt TOPIC message with retain on
  }
  if ( now >= lestimer.mqtt ) {
    //time to uplink a new mqqt message
    console->println("Uplink mqtt");
    lestimer.mqtt = now + MQTTTIMER; //set next time
    meastxt=prtmes(); // publish the data
  }
  if ( now >= lestimer.ttn ) {
  console->println("Uplink TTN");
   lestimer.ttn = now + UPLINKPERIOD; //set next TTN uplink time
   TTNsend(); //issue the TTN message
  }
}


void IRAM_ATTR counter() {
  static unsigned long last_micros = 0;
  unsigned long interrupt_time = millis();
  if ( interrupt_time - last_micros > 50UL ) {
    count += 1;                   // increment count
    }
    last_micros = interrupt_time;
}

void Speed() {
/*
calculate the linear speed from magnets hit count, the routine gets called when average timer lapse (thus after measureperiod ms).
REQUIRES:
count: number of magnet detection
measureperiod: averaging period for rotational counts (ms)
RADIUS= distance between HAL detector and rotation axis centre.

PRVIDES:
sets global variable float speed
*/
  noInterrupts ();
  int interval = measureperiod/1000; //anemometer averaging interval (sec)
  char text[128];  //text buffer for sprintf
  measure data; //data structure for measurements
  int local = count;
  
  int tour = local / 4; //3 magnets, it takes 4 hits to detect a full 360°
  //speed = 3.1415926 * 2 * float(tour) * RADIUS / float(interval) ;

  RPM = float(local) * ( 60 / interval);  // tour/minutes
  speed = RPM * RADIUS * 60 * 3.1415926 * 2. / 1000.; // vitesse radiale km/hr = vitesse angulaire (rad/min) * 2 Pi * rayon (m) * 60 (min/hr)  / 1000 (m/km)
  
  if (local > 0) {
    sprintf(text,"  Speed calculation: Count %d\tRPM %8.2f (T/min)\tspeed %8.4f (km/hr) ",local,RPM, speed);
    console->println(text);
  }
  count=0;
  previousMillis = millis(); //milli of last speed measure used to determine how long to wait until next meaasure
  interrupts ();
}

void setwind(){
  /* et the interrupt mechanism that intercept the magnet detection and accumulates its counting */
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), counter, RISING);
  //timermeasure.set(measureperiod, Speed);
  //TTNuplink.set(ttnuplinkperiod, TTNsend);
  
  
}

String stateDecode(const int16_t result) {
  switch (result) {
  case RADIOLIB_ERR_NONE:
    return "ERR_NONE";
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    return "ERR_CHIP_NOT_FOUND";
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    return "ERR_PACKET_TOO_LONG";
  case RADIOLIB_ERR_RX_TIMEOUT:
    return "ERR_RX_TIMEOUT";
  case RADIOLIB_ERR_CRC_MISMATCH:
    return "ERR_CRC_MISMATCH";
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    return "ERR_INVALID_BANDWIDTH";
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    return "ERR_INVALID_SPREADING_FACTOR";
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    return "ERR_INVALID_CODING_RATE";
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    return "ERR_INVALID_FREQUENCY";
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    return "ERR_INVALID_OUTPUT_POWER";
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
	  return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
  case RADIOLIB_ERR_INVALID_REVISION:
    return "RADIOLIB_ERR_INVALID_REVISION";
  case RADIOLIB_ERR_INVALID_PORT:
    return "RADIOLIB_ERR_INVALID_PORT";
  case RADIOLIB_ERR_NO_RX_WINDOW:
    return "RADIOLIB_ERR_NO_RX_WINDOW";
  case RADIOLIB_ERR_INVALID_CID:
    return "RADIOLIB_ERR_INVALID_CID";
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
  case RADIOLIB_ERR_N_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_A_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
  case RADIOLIB_ERR_NO_JOIN_ACCEPT:
    return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    return "RADIOLIB_LORAWAN_SESSION_RESTORED";
  case RADIOLIB_LORAWAN_NEW_SESSION:
    return "RADIOLIB_LORAWAN_NEW_SESSION";
  case RADIOLIB_ERR_NONCES_DISCARDED:
    return "RADIOLIB_ERR_NONCES_DISCARDED";
  case RADIOLIB_ERR_SESSION_DISCARDED:
    return "RADIOLIB_ERR_SESSION_DISCARDED";
  }
  return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

void debug(bool failed, const __FlashStringHelper* message, int state, bool halt) {
  if(failed) {
    console->print(message);
    console->print(" - ");
    console->print(stateDecode(state));
    console->print(" (");
    console->print(state);
    console->println(")");
    while(halt) { delay(1); }
  }
}

// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len) {
  for(uint16_t c = 0; c < len; c++) {
    char b = buffer[c];
    if(b < 0x10) { console->print('0'); }
    console->print(b, HEX);
  }
  console->println();
}

void TTNsend() {
  uint8_t downlinkPayload[10];  // Make sure this fits your plans!
  size_t  downlinkSize;         // To hold the actual payload size received

  // you can also retrieve additional information about an uplink or
  // downlink by passing a reference to LoRaWANEvent_t structure
  LoRaWANEvent_t uplinkDetails;
  LoRaWANEvent_t downlinkDetails;

  uint8_t fPort = 10;

  // Retrieve the last uplink frame counter
  uint32_t fCntUp = node.getFCntUp();
  // wait before sending another packet

  
  console->print(F("[LoRaWAN] Next uplink in "));
  console->print(delayMs / 1000);
  console->println(F(" seconds\n"));

  // Send a confirmed uplink on the second uplink
  // and also request the LinkCheck and DeviceTime MAC commands


      console->println(F("Sending uplink"));
      if (fCntUp == 1) {
          console->println(F("and requesting LinkCheck and DeviceTime"));
          node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
          node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
          state = node.sendReceive( Payload.getBuffer(), Payload.getSize(), fPort, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails);
      } else {
          state = node.sendReceive(Payload.getBuffer(), Payload.getSize(), fPort, downlinkPayload, &downlinkSize, false, &uplinkDetails, &downlinkDetails);
      }
      uplinkmillis = millis();
      debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);
      console->print("Sending message length ");
      console->println(Payload.getSize());
      // Check if a downlink was received
      // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
      if (state > 0) {
          console->println(F("Received a downlink"));
          // Did we get a downlink with data for us
          if (downlinkSize > 0) {
              console->println(F("Downlink data: "));
              arrayDump(downlinkPayload, downlinkSize);
          } else {
              console->println(F("<MAC commands only>"));
          }

          // print RSSI (Received Signal Strength Indicator)
          console->print(F("[LoRaWAN] RSSI:\t\t"));
          console->print(radio.getRSSI());
          console->println(F(" dBm"));

          // print SNR (Signal-to-Noise Ratio)
          console->print(F("[LoRaWAN] SNR:\t\t"));
          console->print(radio.getSNR());
          console->println(F(" dB"));

          // print extra information about the event
          console->println(F("[LoRaWAN] Event information:"));
          console->print(F("[LoRaWAN] Confirmed:\t"));
          console->println(downlinkDetails.confirmed);
          console->print(F("[LoRaWAN] Confirming:\t"));
          console->println(downlinkDetails.confirming);
          console->print(F("[LoRaWAN] Datarate:\t"));
          console->println(downlinkDetails.datarate);
          console->print(F("[LoRaWAN] Frequency:\t"));
          console->print(downlinkDetails.freq, 3);
          console->println(F(" MHz"));
          console->print(F("[LoRaWAN] Frame count:\t"));
          console->println(downlinkDetails.fCnt);
          console->print(F("[LoRaWAN] Port:\t\t"));
          console->println(downlinkDetails.fPort);
          console->print(F("[LoRaWAN] Time-on-air: \t"));
          console->print(node.getLastToA());
          console->println(F(" ms"));
          console->print(F("[LoRaWAN] Rx window: \t"));
          console->println(state);

          uint8_t margin = 0;
          uint8_t gwCnt = 0;
          if (node.getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
              console->print(F("[LoRaWAN] LinkCheck margin:\t"));
              console->println(margin);
              console->print(F("[LoRaWAN] LinkCheck count:\t"));
              console->println(gwCnt);
          }

          uint32_t networkTime = 0;
          uint8_t fracSecond = 0;
          if (node.getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
              console->print(F("[LoRaWAN] DeviceTime Unix:\t"));
              console->println(networkTime);
              console->print(F("[LoRaWAN] DeviceTime second:\t1/"));
              console->println(fracSecond);
          }

      } else {
          console->println(F("[LoRaWAN] No downlink received"));
      }
    } //TTNsend
