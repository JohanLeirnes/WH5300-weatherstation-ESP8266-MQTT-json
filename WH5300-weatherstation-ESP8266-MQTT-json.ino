/*
Created by Johan Lindström <johan.von.lindstrom@gmail.com>
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation.
Libraries :
 - ESP8266 core for Arduino : https://github.com/esp8266/Arduino
 - PubSubClient : https://github.com/knolleary/pubsubclient
 - DHT : https://github.com/adafruit/DHT-sensor-library
 - ArduinoJson : https://github.com/bblanchon/ArduinoJson
 - WiFiManager : https://github.com/tzapu/WiFiManager/releases

 modified by Laserlicht
*/

//IMPORTANT: Board must be configured with SPIFFS before compiling and flashing!!!

#include <FS.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>  

#define SERIALDEBUG // Comment this out with a // for the final upload.

#define MQTT_VERSION MQTT_VERSION_3_1_1
#define MQTT_MAX_PACKET_SIZE 512 //important change in header file

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
int port = 23;

// Wifi: SSID and password
const char* host = "Wetterstation-WH5300";

// MQTT: ID, server IP, port, username and password
// Disable Wifi-Access-Point & Restart Device to reconfigure
char MQTT_CLIENT_ID[40] = "v-station";
char MQTT_SERVER_IP[20] = "192.168.XXX.XXX";
char MQTT_SERVER_PORT[6] = "1883";
char MQTT_USER[40] = "XXXX";
char MQTT_PASSWORD[40] = "XXXX";

// MQTT: topic
char MQTT_SENSOR_TOPIC[100] = "v-station/sensor1"; //this is where all things except rain is reported
char MQTT_SENSOR_TOPIC2[100] = "v-station/sensor2"; //this is where rain is reported

//IP
char static_ip[16] = "10.0.1.56";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";

#ifdef SERIALDEBUG
#define debug(x)     Serial.print(x)
#define debugln(x)   Serial.println(x)
#else
#define debug(x)     Telnet.print(x)
#define debugln(x)   Telnet.println(x)
#endif

WiFiClient wifiClient;
PubSubClient client(wifiClient);

WiFiServer TelnetServer(port);
WiFiClient Telnet;

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long previousMillis3 = 0;
unsigned long interval = 0;
// Set the interval in times you want to skip measuring.
// The weatherstation reports once every 48sec

int pushButton = 14;
boolean intro=0;
boolean dataBuff[500];
int datasent=0;
byte byteArray[15];
boolean ldataBuff[500]; //received bits buffer
int firstcheckdone=0;
int firstraincheckdone=0;
int lp;





//WiFiManager
//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void init_wifimanager() {
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        auto error = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if (!error) {
          Serial.println("\nparsed json");

          strcpy(MQTT_CLIENT_ID, json["MQTT_CLIENT_ID"]);
          strcpy(MQTT_SERVER_IP, json["MQTT_SERVER_IP"]);
          strcpy(MQTT_SERVER_PORT, json["MQTT_SERVER_PORT"]);
          strcpy(MQTT_USER, json["MQTT_USER"]);
          strcpy(MQTT_PASSWORD, json["MQTT_PASSWORD"]);
          strcpy(MQTT_SENSOR_TOPIC, json["MQTT_SENSOR_TOPIC"]);
          strcpy(MQTT_SENSOR_TOPIC2, json["MQTT_SENSOR_TOPIC2"]);

          if(json["ip"]) {
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
          }

        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_MQTT_CLIENT_ID("id", "client id", MQTT_CLIENT_ID, 40);
  WiFiManagerParameter custom_MQTT_SERVER_IP("mqttip", "server ip", MQTT_SERVER_IP, 20);
  WiFiManagerParameter custom_MQTT_SERVER_PORT("port", "server port", MQTT_SERVER_PORT, 6);
  WiFiManagerParameter custom_MQTT_USER("user", "user", MQTT_USER, 40);
  WiFiManagerParameter custom_MQTT_PASSWORD("password", "password", MQTT_PASSWORD, 40);
  WiFiManagerParameter custom_MQTT_SENSOR_TOPIC("topic", "topic", MQTT_SENSOR_TOPIC, 100);
  WiFiManagerParameter custom_MQTT_SENSOR_TOPIC2("topic2", "topic2", MQTT_SENSOR_TOPIC2, 100);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  IPAddress _ip,_gw,_sn;
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);

  //add all your parameters here
  wifiManager.addParameter(&custom_MQTT_CLIENT_ID);
  wifiManager.addParameter(&custom_MQTT_SERVER_IP);
  wifiManager.addParameter(&custom_MQTT_SERVER_PORT);
  wifiManager.addParameter(&custom_MQTT_USER);
  wifiManager.addParameter(&custom_MQTT_PASSWORD);
  wifiManager.addParameter(&custom_MQTT_SENSOR_TOPIC);
  wifiManager.addParameter(&custom_MQTT_SENSOR_TOPIC2);
 

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("WeatherStationAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(MQTT_CLIENT_ID, custom_MQTT_CLIENT_ID.getValue());
  strcpy(MQTT_SERVER_IP, custom_MQTT_SERVER_IP.getValue());
  strcpy(MQTT_SERVER_PORT, custom_MQTT_SERVER_PORT.getValue());
  strcpy(MQTT_USER, custom_MQTT_USER.getValue());
  strcpy(MQTT_PASSWORD, custom_MQTT_PASSWORD.getValue());
  strcpy(MQTT_SENSOR_TOPIC, custom_MQTT_SENSOR_TOPIC.getValue());
  strcpy(MQTT_SENSOR_TOPIC2, custom_MQTT_SENSOR_TOPIC2.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument json(1024);
    json["MQTT_CLIENT_ID"] = MQTT_CLIENT_ID;
    json["MQTT_SERVER_IP"] = MQTT_SERVER_IP;
    json["MQTT_SERVER_PORT"] = MQTT_SERVER_PORT;
    json["MQTT_USER"] = MQTT_USER;
    json["MQTT_PASSWORD"] = MQTT_PASSWORD;
    json["MQTT_SENSOR_TOPIC"] = MQTT_SENSOR_TOPIC;
    json["MQTT_SENSOR_TOPIC2"] = MQTT_SENSOR_TOPIC2;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
    //end save
  }
}




// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
}

void handleTelnet(){
  if (TelnetServer.hasClient()){
  	// client is connected
    if (!Telnet || !Telnet.connected()){
      if(Telnet) Telnet.stop();          // client disconnected
      Telnet = TelnetServer.available(); // ready for new client
    } else {
      TelnetServer.available().stop();  // have client, block new conections
    }
  }
  if (Telnet && Telnet.connected() && Telnet.available()){
  // client input processing
  while(Telnet.available())
    Serial.write(Telnet.read()); // pass through
    // do other stuff with client input here
  }
}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    debug("INFO: Ansluter till MQTT servern...");
    const char* user = MQTT_USER;
    const char* password = MQTT_PASSWORD;
    if(user=="none") user = NULL;
    if(password=="none") password = NULL;
    if (client.connect(MQTT_CLIENT_ID, user, password)) {
      debugln("INFO: ansluten");
    } else {
      debug("ERROR: gick ej ansluta, rc=");
      debug(client.state());
      debugln("DEBUG: försöker igen om 5 sekunder");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  TelnetServer.begin();
  delay(100);
  pinMode(pushButton, INPUT);
  delay(10);
  debugln();
  debugln();
  debug("INFO: Ansluter till ");
  init_wifimanager();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug(".");
  }
  debugln("");
  debugln("INFO: WiFi ansluten");
  debugln("INFO: IP adress: ");
  debugln(WiFi.localIP());

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, atoi(MQTT_SERVER_PORT));
  client.setCallback(callback);

  MDNS.begin(host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
}
int old,last;
unsigned long dur;
float temp,wSpeed,wGust,rAcum,rAcumold,rAcumpub;
int  aux,id,hum,unk,status,dir;
int p=0,b=0,i=0;
static const char* const windDirections[] = {"N","NE","E","SE","S","SW","W","NW"};

int crc8(boolean *BitString,int nBits)
{
        char CRC=0;
        int  i;
        boolean DoInvert;
        for (i=0; i<nBits; ++i)
        {
                DoInvert = (BitString[i] ^ (CRC&0x80)>>7);
                if(DoInvert){
                        CRC=CRC^0x18;
                }
                CRC=CRC<<1;
                if(DoInvert){
                        CRC=CRC|0x01;
                }
        }
        return(CRC);
}

void decode(unsigned char byteArray[8]){
  int type=(byteArray[0]&0xF0)>>4;
  switch (type){
          case 0x0A: //Weather message
                  //Getting the data
                  id=((byteArray[0]&0x0F)<<4)+ ((byteArray[1]&0xF0)>>4);
                  aux=((byteArray[1]&0x0F)<<8)+ ((byteArray[2]&0xFF));
                  temp=(aux*0.1)-40;
                  hum=byteArray[3];
                  aux=byteArray[4];
                  wSpeed=(aux/3.6);
                  aux=byteArray[5];
                  wGust=(aux/3.6);
                  unk=(byteArray[6]&0xF0)>>4;
                  aux=((byteArray[6]&0x0F)<<8)+ ((byteArray[7]&0xFF));
                  rAcum=(((aux/3)*0.01)*25.4);
                  status=(byteArray[8]&0xF0)>>4;
                  dir=(byteArray[8]&0x0F)>>1;
                  break;
          default:
                  debugln("Unknown message: " + String(type));
                  intro=1;
                  p=0;
                  break;

  }
  if(firstcheckdone==0){
    rAcumold=rAcum;
  }
  if(firstcheckdone==0 && hum != 0){
    firstcheckdone=1;
    debugln("First check done");
  }
  if(firstraincheckdone==1){
    rAcumold=rAcum;
    firstraincheckdone=0;
  }
}

void publishData(float temp,int hum, float rAcum, float wSpeed,float wGust,int dir,int status) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonDocument<200> root;
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = String(temp);
  root["humidity"] = String(hum);
  root["rainacum"] = String(rAcum);
  root["wind"] = String(wSpeed);
  root["windgust"] = String(wGust);
  root["winddir"] = String(windDirections[dir]);
  root["status"] = String(status);
  debugln("Temperatur: " + String(temp) + " ºC");
  debugln("Luftfuktighet: " + String(hum) + " %");
  debugln("Rain (acum): " + String(rAcum) + " mm");
  debugln("Vindhastighet: " + String(wSpeed) + " m/s");
  debugln("Vindbyar: " + String(wGust) + "m/s");
  debugln("Status bits: " + String(status));
  debugln("Vindriktning: " + String(windDirections[dir]));
  char data[200];
  serializeJson(root, data);
  client.publish(MQTT_SENSOR_TOPIC, data, true);
  datasent=1;
  debugln("Data sent, taking a pause");
  intro=1;
  p=0;
  debugln("p:" + String(p));
}

void publishDatarain(float rAcumpub) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonDocument<200> root2;
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root2["rain"] = String(rAcumpub);
  debugln("Regn senaste 15minuter: " + String(rAcumpub) + " mm");
  char data2[200];
  serializeJson(root2, data2);
  client.publish(MQTT_SENSOR_TOPIC2, data2, true);
  debugln("First raincheck done:");
}

void loop() {
  handleTelnet();
  int buttonState = digitalRead(pushButton);
  if(datasent == 0){
    if (buttonState != old && p < 500) {
      if((old==1) && (micros() - dur)<=800){
        dataBuff[p++]=1;
        intro=0;
      }else if(old==1){
        dataBuff[p++]=0;
        intro=0;
      }
      old=buttonState;
      dur=micros();
    }else if(p >= 500||(micros() - dur)>=10000){
      //debugln("To much data, restarting loop!");
      p=0;
      intro=1;
      //delay(1000);
      return;
    }
  }
  if(datasent == 0 && p >= 80){
    if((micros() - dur)>5000 && intro==0){
      debugln();
      debugln("p:" + String(p));
      debugln();
      /*for(int i = 0; i < p; i++)
      {
        debug(" " + String(dataBuff[i]) + " |");
      }*/
      if(p>255){
        debugln("To much data, restarting loop!");
        p=0;
        intro=1;
        return;
      }
      lp=p;
      b=0;
      for(i=(p-80);i<lp;i++){
        byteArray[b]=byteArray[b]*2+dataBuff[i];
        if(((i-lp-80)+1)%8==0) {
          b++;
          byteArray[b]=0;
        }
      }
      debugln();
    if(crc8(&dataBuff[p-80],80)==0){ //CRC OK
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis2 >= 880000){
        firstraincheckdone=1;
        decode(byteArray);
      }
      else{
        decode(byteArray);
      }
    }
    else{
       debugln("CRC Error");
       p=0;
       intro=1;
       debugln("p after CRC check reset:" + String(p));
       return;
    }
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    httpServer.handleClient();
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis2 >= 900000 && hum != 0) {
      previousMillis2 = currentMillis;
      rAcumpub=rAcum-rAcumold;
      publishDatarain(rAcumpub);
    }
    if(currentMillis - previousMillis >= 1000 && hum != 0){
      previousMillis = currentMillis;
      publishData(temp, hum, rAcum, wSpeed, wGust, dir, status);
      }
  }
}
  unsigned long currentMillis2 = millis();
  if(currentMillis2 - previousMillis3 >= interval*48000 + 47500 && datasent == 1) {
  debugln("Pause done, lets go again!");
  debugln("-------------------------------------------------------");
  previousMillis3 = currentMillis2;
  datasent=0;
  }
}
