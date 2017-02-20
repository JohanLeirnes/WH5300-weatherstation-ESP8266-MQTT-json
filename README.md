# WH5300-weatherstation-ESP8266-MQTT-json
A ESP8266 based sniffer for the WH5300 weather station

IMPORTANT:
Add a 470uF Electrolytic-cap and a 0.1uF Ceramic-cap between your esp8266 powerpins if you are getting resets on your esp8266.
Also be shure to make good solder connections to the reciever.

Connect your GPIO14 to the Weatherstations receiver (the LCD-screen) DOUT pin and the GND pin to the receivers GND pin.
There are 3 pins on the device. I have marked out the GND pin and DOUT pin on the Receiver:

![alt tag](https://github.com/bhaap/WH5300-weatherstation-ESP8266-MQTT-json/raw/master/Receiver-pins.png)

If you are getting good results upload the sketch with #define DEBUG commented out (// infront)
