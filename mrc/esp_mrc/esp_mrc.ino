#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include "shared.h"

void syncData(String *);

#define UDP_PORT 1337

long long last_update = 0;
#define UPDATE_RATE_MS 1000

WiFiUDP UDP;

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("#");
}

void loop()
{
  if (Serial.available())
  {
    processSerialData();
  }

  if( (millis() - last_update) > UPDATE_RATE_MS){
    last_update = millis();
    syncData(0);
  }
}

void syncData(String *data = nullptr)
{
  HTTPClient http;

  http.begin(ROS_HOST);
  int returnCode = http.GET();

  if (returnCode < 0)
  {
    Serial.print("!");
    Serial.println(http.errorToString(returnCode));
  }
  else
  {
    String pose_estimation = http.getString();
    Serial.println(pose_estimation); //send it to arduino
  }
}

void processSerialData()
{
  bool send = false;
  char byte = Serial.read();
  switch (byte)
  {
  case 'u':
  case 'l':
  case 'r':
  case 'd':
  case 's':
  send = true;
  break;
  default:
  send = false;
    break;
  }

  if (send)
  {
    UDP.beginPacket("10.0.100.119",1337);
    UDP.write(byte);
    UDP.endPacket();
    //syncData(&button);
  }

}
