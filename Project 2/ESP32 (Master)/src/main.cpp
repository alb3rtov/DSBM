#include <Arduino.h>
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define XPin 34
#define YPin 35
#define SWPin 27

QueueHandle_t xMutex;
BluetoothSerial BT;
String clientName = "ESP32_Servomotor";
bool connected;
int ADC_Max = 4096;

/* Configure and connect bluetooth */
void bluetoothConnect() {
  BT.begin("ESP32_client", true); /* True indicates that is configured as a master */
  Serial.println("El dispositivo Bluetooth está en modo maestro. Conectando con el anfitrión ...");
  connected = BT.connect(clientName);
  if(connected) {
    Serial.println("¡Conectado exitosamente!");
  } else {
    while(!BT.connected(10000)) {
      Serial.println("No se pudo conectar. Asegúrese de que el dispositivo remoto esté disponible y dentro del alcance, luego reinicie la aplicación."); 
    }
  }
}

/* Send values of joystick to the other ESP32 */
void sendInfoToSlave(int val, int pin) {
  xSemaphoreTake( xMutex, 500/portTICK_PERIOD_MS  ); /* Use a mutex to avoid multiple tasks in this section of code */
  {
    /* Send two data in order to identify if the data is from X or Y position (determined by pin variable) */
    BT.write(val);
    BT.write(pin+180);
  }
}

/* Read a given position (X or Y) of the joystick */
void readPinTask(void *pvParameters) {
  int val = 0;
  int pin = ( int ) pvParameters;
  for(;;) {
    val = analogRead(pin);
    val = map(val, 0, ADC_Max, 0, 180); /* Map analog reading to 0 to 180 degrees */
    sendInfoToSlave(val, pin);
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void app_main() {
  xMutex = xSemaphoreCreateMutex();

  if( xMutex != NULL )
  {
    xTaskCreate(readPinTask, "XPin", 4096, (void *) XPin, 10, NULL);
    xTaskCreate(readPinTask, "YPin", 4096, (void *) YPin, 10, NULL);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SWPin, INPUT_PULLUP);
  pinMode(YPin, INPUT);
  pinMode(XPin, INPUT);

  bluetoothConnect();
  app_main();
}

void loop() {

}
