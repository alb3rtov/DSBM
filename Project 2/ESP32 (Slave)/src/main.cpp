#include <Arduino.h>
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define pinServo1 18
#define pinServo2 19
#define XPin 34
#define YPin 35

BluetoothSerial BT; // Objeto Bluetooth
Servo myServo1;
Servo myServo2;

void initServo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo1.setPeriodHertz(50);
  myServo1.attach(pinServo1, 500, 2400);
  myServo2.setPeriodHertz(50);
  myServo2.attach(pinServo2, 500, 2400);

  myServo1.write(90);
  myServo2.write(90);
}

void readIncomingData(void *pvParameters) {
  int incoming = 0;
  int previousData = 0;
  for(;;) {
    if (BT.available()) {
      incoming = BT.read();
      if (incoming == XPin+180) {
        Serial.print("Recibido (X): ");
        Serial.println(previousData);
        myServo1.write(previousData);
      } else if (incoming == YPin+180) {
        Serial.print("Recibido (Y): ");
        Serial.println(previousData);
        myServo2.write(previousData);
      } else {
        previousData = incoming;
      }
    }
    vTaskDelay(150/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}


void app_main() {
  xTaskCreate(readIncomingData, "Read data", 4096, NULL, 10, NULL);
}

void setup() {
  Serial.begin(9600); // Inicialización de la conexión en serie para la depuración
  BT.begin("ESP32_Servomotor"); // Nombre de su dispositivo Bluetooth y en modo esclavo
  Serial.println("El dispositivo Bluetooth está listo para emparejarse");
  initServo();

  app_main();
}

void loop() {

}