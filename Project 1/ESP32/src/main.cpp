#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "ArduinoJson.h"
#include <string>
#include <vector>
#include <sstream>

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define TOPIC1 "esi/room1/sensors"
#define TOPIC2 "esi/room2/sensors"
#define BROKER_IP "192.168.43.84"
#define BROKER_PORT 2883 

const char* ssid = "AndroidAP";
const char* password = "password";
static const uart_port_t uart_num = UART_NUM_2;

WiFiClient espClient;
PubSubClient client(espClient);

/* Connect to Wi-Fi AP */
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

/* Connect to mqtt broker */
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

/* Convert string to json object and send info to mqtt topic */
void send_info_mqtt(String data_char) {
  std::string segment;
  std::stringstream stream(data_char.c_str());
  std::vector<std::string> seglist;
  
  /* Split the receive string data */
  while(std::getline(stream, segment, ':'))
  {
    seglist.push_back(segment);
  }
  
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder[seglist.at(0).c_str()] = atoi(seglist.at(1).c_str());
  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  if (JSONencoder.containsKey("temperature") || JSONencoder.containsKey("brightness")) {
    Serial.println("Send to topic 1");
    client.publish(TOPIC1, JSONmessageBuffer);
  } else {
    Serial.println("Send to topic 2");
    client.publish(TOPIC2, JSONmessageBuffer);
  }
}

/* Receive data from STM32 */
void receive_data(void *pvParameters) {
  for(;;) {
    String data_string;
    const uart_port_t uart_num = UART_NUM_2;
    uint8_t data[128];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    length = uart_read_bytes(uart_num, data, length, 100);
    if (length > 0) {
        data_string = (char*) data;
        Serial.println(data_string);
        send_info_mqtt(data_string);
    }
    vTaskDelay(150/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void app_main() {
  xTaskCreate(receive_data, "Read data", 4096, NULL, 10, NULL);
}

void setup() {
    Serial.begin(115200);
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, 18, 19));
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));
    delay(4000);
    wifiConnect();
    mqttConnect();

    app_main();
}

void loop() {

}