#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

#define RXD2 16
#define TXD2 17

#define TOPIC "esi/dsbm"
#define BROKER_IP "192.168.43.174"
#define BROKER_PORT 1883 

const char* ssid = "AndroidAP";
const char* password = "password";

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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(4000);
  wifiConnect();
  mqttConnect();
}

void loop() {
  if(Serial2.available()){ 
    const char* payload = Serial2.readString().c_str();
    client.publish(TOPIC, payload);
  }
  delay(2000);
}





