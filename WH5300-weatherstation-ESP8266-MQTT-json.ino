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

#define MQTT_VERSION MQTT_VERSION_3_1_1

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;


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
const char* MQTT_SENSOR_TOPIC = "v-station/sensor1";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;

int pushButton = 14;
boolean intro=0;
boolean dataBuff[255];
byte byteArray[15];
boolean ldataBuff[255]; //received bits buffer
boolean firstcheckdone=0;
int lp;
boolean DEBUG=1; //Set this flag to 0 for final upload

// function called to publish the temperature and the humidity


// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if(DEBUG=1){
    Serial.print("INFO: Ansluter till MQTT servern...");
  }
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      if(DEBUG=1){
      Serial.println("INFO: ansluten");
    }
    } else {
      if(DEBUG=1){
      Serial.print("ERROR: gick ej ansluta, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: försöker igen om 5 sekunder");
    }
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  if(DEBUG=1){
  Serial.begin(9600);
  }
  pinMode(pushButton, INPUT);
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
  delay(10);
  if(DEBUG=1){
  Serial.println();
  Serial.println();
  Serial.print("INFO: Ansluter till ");
  Serial.println(WIFI_SSID);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if(DEBUG=1){
  Serial.println("");
  Serial.println("INFO: WiFi ansluten");
  Serial.println("INFO: IP adress: ");
  Serial.println(WiFi.localIP());
  }

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);

  MDNS.begin(host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
}
int old;
unsigned long dur;
float temp,wSpeed,wGust,rAcum,rAcumold,rAcumpub;
int  aux,id,hum,unk,status,dir;
int p=0,b=0,i=0,j=0;
static const char* const windDirections[] = {"N","NE","E","SE","S","SW","W","NW"};

void decode(unsigned char byteArray[8]){
  if(firstcheckdone=1){
    rAcumold=rAcum;
  }
  int type=(byteArray[0]&0xF0)>>4;
  switch (type){
          case 0x0A: //Weather message
                  //Getting the data
                  id=((byteArray[0]&0x0F)<<4)+ ((byteArray[2]&0xF0)>>4);
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
                  //Serial.println("Unknown message: " + String(type));
                  break;
  }
  if(firstcheckdone=0){
    rAcumold=rAcum;
  }
}

void publishData(float temp,int hum,float wSpeed,float wGust,int dir,int status) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = (String)temp;
  root["humidity"] = (String)hum;
  root["wind"] = (String)wSpeed;
  root["windgust"] = (String)wGust;
  root["winddir"] = (String)windDirections[dir];
  root["status"] = (String)status;
  if(DEBUG=1){
  Serial.println("Temperatur: " + String(temp) + " ºC");
  Serial.println("Luftfuktighet: " + String(hum) + " %");
  Serial.println("Vindhastighet: " + String(wSpeed) + " m/s");
  Serial.println("Vindbyar: " + String(wGust) + "m/s");
  Serial.println("Status bits: " + String(status));
  Serial.println("Vindriktning: " + String(windDirections[dir]));
  }
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(MQTT_SENSOR_TOPIC, data, true);
}

void publishDatarain(float temp,int hum,float wSpeed,float wGust,int dir,int status,float rAcumpub) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  rAcumpub=rAcum-rAcumold;
  root["temperature"] = (String)temp;
  root["humidity"] = (String)hum;
  root["wind"] = (String)wSpeed;
  root["windgust"] = (String)wGust;
  root["rain"] = (String)rAcumpub;
  root["winddir"] = (String)windDirections[dir];
  root["status"] = (String)status;
  if(DEBUG=1){
  Serial.println("Temperatur: " + String(temp) + " ºC");
  Serial.println("Luftfuktighet: " + String(hum) + " %");
  Serial.println("Vindhastighet: " + String(wSpeed) + " m/s");
  Serial.println("Vindbyar: " + String(wGust) + "m/s");
  Serial.println("Akumulerat regn: " + String(rAcum - rAcumold) + " mm");
  Serial.println("Status bits: " + String(status));
  Serial.println("Vindriktning: " + String(windDirections[dir]));
  }
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(MQTT_SENSOR_TOPIC, data, true);
}

void loop() {
  // Läs ingångspinnen :
  ESP.wdtFeed();
  int buttonState = digitalRead(pushButton);
  if (buttonState != old) {  // skriva ut tillståndet för knappen:
    if((old==1) && (micros() - dur)<800){
      dataBuff[p++]=1;
      intro=0;
    }else if(old==1 && (micros() - dur)>=800){
      dataBuff[p++]=0;
      intro=0;
    }
   old=buttonState;
   dur=micros();
   if(p<=80){
     return;
   }
  }
  if((micros() - dur)>50000 && intro==0){
    if(p>80){
      firstcheckdone=1;
      lp=p;
      memcpy(ldataBuff,dataBuff,sizeof(ldataBuff));
      b=0;
      for(i=(p-80);i<lp;i++){
        byteArray[b]=byteArray[b]*2+ldataBuff[i];
        if(((i-lp-80)+1)%8==0) {
          b++;
          byteArray[b]=0;
        }
      }
      if(DEBUG=1){
      Serial.println();
      Serial.print(b);
      Serial.print(" bytes:");
      for(j=0;j<b;j++){
       printHex(byteArray[j],2);
       Serial.print(" ");
      }
      Serial.println();
      }
      decode(byteArray);
    }
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    httpServer.handleClient();
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis2 >= 900000 && hum != 0) {
      previousMillis2 = currentMillis;
      publishDatarain(temp, hum, wSpeed, wGust, dir, status, rAcumpub);
    }
    else if(currentMillis - previousMillis >= 5000 && hum != 0){
      previousMillis = currentMillis;
      publishData(temp, hum, wSpeed, wGust, dir, status);
    }
    intro=1;
    p=0;
  }
}

void printHex(int num, int precision) {
     char tmp[16];
     char format[128];

     sprintf(format, "0x%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
}
