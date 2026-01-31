#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include <math.h>

// --- Config ---
#define JOY_X_PIN 34  
#define JOY_Y_PIN 35  // *** ต้องต่อสายนี้ ค่าถึงจะหมุนรอบ ***
#define JOY_SW_PIN 23 

const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-joy-sender-smooth"; 
const char MQTT_TOPIC[] = "esp32/command";

WiFiClient network;
MQTTClient mqtt(256);
QueueHandle_t mqttQueue;

struct JoyMessage { char type; int value; };
bool inAutoMode = false;
int lastSentAngle = -1;

// ตัวแปรสำหรับทำ Smoothing (กรองค่าให้นิ่ง)
float smoothX = 0;
float smoothY = 0;
float alpha = 0.1; // ค่าความหน่วง (0.1 = นุ่มมาก, 0.5 = ไว)

void connectToMQTT() {
  while (!mqtt.connect(MQTT_CLIENT_ID)) vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void mqttTask(void *parameter) {
  mqtt.begin(MQTT_BROKER_ADRRESS, network);
  connectToMQTT();
  JoyMessage rcvMsg;
  char payload[10];
  while (true) {
    if (!mqtt.connected()) connectToMQTT();
    mqtt.loop();
    if (xQueueReceive(mqttQueue, &rcvMsg, 0) == pdTRUE) {
      if (rcvMsg.type == 'J') {
        sprintf(payload, "J%d", rcvMsg.value); 
        mqtt.publish(MQTT_TOPIC, payload);
      } else if (rcvMsg.type == 'A') mqtt.publish(MQTT_TOPIC, "AUTO");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void joystickTask(void *parameter) {
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  
  // กำหนดค่าเริ่มต้น
  smoothX = analogRead(JOY_X_PIN);
  smoothY = analogRead(JOY_Y_PIN);

  while (true) {
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    // --- 1. Smoothing Process (ทำให้ค่านุ่มนวล) ---
    // สูตร: ค่าใหม่ = (ค่าเก่า * 0.9) + (ค่าปัจจุบัน * 0.1)
    smoothX = (smoothX * (1.0 - alpha)) + (rawX * alpha);
    smoothY = (smoothY * (1.0 - alpha)) + (rawY * alpha);

    // เช็คปุ่ม AUTO
    if (digitalRead(JOY_SW_PIN) == LOW) {
      inAutoMode = !inAutoMode;
      JoyMessage msg; msg.type = 'A';
      xQueueSend(mqttQueue, &msg, 0);
      vTaskDelay(500 / portTICK_PERIOD_MS); 
    }

    if (!inAutoMode) {
      // ใช้ค่า smooth แทนค่า raw
      int mapX = (int)smoothX - 2048; 
      int mapY = (int)smoothY - 2048;

      // Deadzone: ถ้าอยู่ตรงกลาง (ระยะห่าง < 500) ไม่ต้องส่งค่า
      if (sqrt((mapX*mapX) + (mapY*mapY)) > 500) {
        
        // --- 2. คำนวณองศารอบตัว (360) ---
        double radian = atan2(mapY, mapX);
        int angle = radian * (180.0 / PI);

        // ปรับทิศทาง (Offset) ให้ตรงกับหน้าจอ
        // ลองเปลี่ยน +270 เป็นค่าอื่นถ้าทิศไม่ตรง (เช่น +90, +180)
        angle = (angle + 270) % 360; 
        if (angle < 0) angle += 360;

        // ส่งค่าเมื่อมีการเปลี่ยนแปลง
        // ลด threshold เหลือ 2 เพื่อให้ละเอียดขึ้น
        if (abs(angle - lastSentAngle) > 2) {
          JoyMessage msg; msg.type = 'J'; msg.value = angle;
          xQueueSend(mqttQueue, &msg, 0);
          lastSentAngle = angle;
        }
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); // อ่านค่าถี่ขึ้น (20ms)
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  mqttQueue = xQueueCreate(10, sizeof(JoyMessage));
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(joystickTask, "Joy", 4096, NULL, 1, NULL);
}

void loop() {}