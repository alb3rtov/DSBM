#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <math.h>
#include "ArduinoJson.h"

const int B = 4275;
const char* ssid = "AndroidAP";
const char* password = "password";

#define TOPIC "esi/ld4"
#define BROKER_IP "192.168.43.174"
#define BROKER_PORT 1883 

const int ledPin1=36;
const int ledPin2=39;

WiFiClient espClient;
PubSubClient client(espClient);

void wifiConnect()
{
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void mqttConnect() {
  // TBD
  client.setServer(BROKER_IP, BROKER_PORT);
  
  while (!client.connected()) {
    client.connect("ESP32-avm");
    Serial.printf("MQTT connecting ...");
    delay(5000);
  }
  Serial.printf("connected");
}

int measureLight() {
  int analogValue = analogRead(ledPin1);
  int lightPercentage = 0;
  
  if (analogValue <= 0) {
    lightPercentage = 0;
  } else if (analogValue < 50) {
    lightPercentage = 15;
  } else if (analogValue < 200) {
    lightPercentage = 20;
  } else if (analogValue < 350) {
    lightPercentage = 25;
  } else if (analogValue < 500) {
    lightPercentage = 30;
  } else if (analogValue < 650) {
    lightPercentage = 35;
  } else if (analogValue < 800) {
    lightPercentage = 40;
  } else if (analogValue < 950) {
    lightPercentage = 45;
  } else if (analogValue < 1100) {
    lightPercentage = 55;
  } else if (analogValue < 1250) {
    lightPercentage = 60;
  } else if (analogValue < 1400) {
    lightPercentage = 65;
  } else if (analogValue < 1550) {
    lightPercentage = 40;
  } else if (analogValue < 1700) {
    lightPercentage = 75;
  } else if (analogValue < 1850) {
    lightPercentage = 80;
  } else if (analogValue < 2000) {
    lightPercentage = 85;
  } else if (analogValue < 2150) {
    lightPercentage = 90;
  } else if (analogValue < 2300) {
    lightPercentage = 95;
  } else {
    lightPercentage = 100;
  }
  
  return lightPercentage;
}

int measureTemperature() {
  int a = analogRead(ledPin2);

  float R = 1023.0/(((float)a)-1.0);
  R = 100000.0*R;
  float temperature=1.0/(log(R/100000.0)/B+1/298.15)-273.15;

  return temperature;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ledPin1,OUTPUT);
  delay(4000);
  wifiConnect();
  mqttConnect();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();

  int brightnessMeasurament = measureLight();
  int temperatureMeasurament = measureTemperature();

  JSONencoder["brightness"] = brightnessMeasurament;
  JSONencoder["temperature"] = temperatureMeasurament;

  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

  client.publish(TOPIC, JSONmessageBuffer);
  delay(2000);
}
