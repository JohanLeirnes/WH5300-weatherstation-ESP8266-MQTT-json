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

*/
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

//#define DEBUG
#define MQTT_VERSION MQTT_VERSION_3_1_1

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
int port = 23;

// Wifi: SSID and password
const char* host = "v-station-webupdate";
const char* WIFI_SSID = "XXXXXX";
const char* WIFI_PASSWORD = "XXXXX";

// MQTT: ID, server IP, port, username and password
const char* MQTT_CLIENT_ID = "v-station";
const char* MQTT_SERVER_IP = "192.168.XXX.XXX";
const uint16_t MQTT_SERVER_PORT = 1883;
const char* MQTT_USER = "XXXX";
const char* MQTT_PASSWORD = "XXXX";

// MQTT: topic
const char* MQTT_SENSOR_TOPIC = "v-station/sensor1"; //this is where all things except rain is reported
const char* MQTT_SENSOR_TOPIC2 = "v-station/sensor2"; //this is where rain is reported

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
boolean dataBuff[255];
int datasent=0;
byte byteArray[15];
boolean ldataBuff[255]; //received bits buffer
int firstcheckdone=0;
int firstraincheckdone=0;
int lp;

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
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
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
  debugln(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug(".");
  }
  debugln("");
  debugln("INFO: WiFi ansluten");
  debugln("INFO: IP adress: ");
  debugln(WiFi.localIP());

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
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

void publishData(float temp,int hum,float wSpeed,float wGust,int dir,int status) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = String(temp);
  root["humidity"] = String(hum);
  root["wind"] = String(wSpeed);
  root["windgust"] = String(wGust);
  root["winddir"] = String(windDirections[dir]);
  root["status"] = String(status);
  debugln("Temperatur: " + String(temp) + " ºC");
  debugln("Luftfuktighet: " + String(hum) + " %");
  debugln("Vindhastighet: " + String(wSpeed) + " m/s");
  debugln("Vindbyar: " + String(wGust) + "m/s");
  debugln("Status bits: " + String(status));
  debugln("Vindriktning: " + String(windDirections[dir]));
  char data[200];
  root.printTo(data, root.measureLength() + 1);
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
  StaticJsonBuffer<200> jsonBuffer2;
  JsonObject& root2 = jsonBuffer2.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root2["rain"] = String(rAcumpub);
  debugln("Regn senaste 15minuter: " + String(rAcumpub) + " mm");
  char data[200];
  root2.printTo(data, root2.measureLength() + 1);
  client.publish(MQTT_SENSOR_TOPIC2, data, true);
  debugln("First raincheck done:");
}

void loop() {
  handleTelnet();
  int buttonState = digitalRead(pushButton);
  if(datasent == 0){
    if (buttonState != old) {
      if((old==1) && (micros() - dur)<=800){
        dataBuff[p++]=1;
        intro=0;
        /*if(last=0){
        debugln(1);
        }
        else{
          debug(1);
        }
        last=1;*/
      }else if(old==1){
        dataBuff[p++]=0;
        intro=0;
        /*if(last=1){
        debugln(0);
        }
        else{
          debug(0);
        }
        last=0;*/
      }
      old=buttonState;
      dur=micros();
      }
    }
  if(datasent == 0 && p >= 80){
    if((micros() - dur)>50000 && intro==0){
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
      publishData(temp, hum, wSpeed, wGust, dir, status);
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
