#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// กำหนดขา
#define JOY_X_PIN 34  // ต่อ VRX ของจอยสติ๊ก
#define JOY_SW_PIN 23 // (Option) ต่อขา SW ของจอย เพื่อกดปุ่มกลับโหมด Auto

const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-joy-sender-001"; 
const char MQTT_TOPIC[] = "esp32/command"; // ส่งไปหัวข้อเดียวกับที่ตัวรับรอฟัง

WiFiClient network;
MQTTClient mqtt(256);
QueueHandle_t mqttQueue;

// Struct เพื่อเก็บข้อมูลที่จะส่งใน Queue
struct JoyMessage {
  char type; // 'J' = Angle, 'A' = Auto Command
  int value;
};

void connectToMQTT() {
  while (!mqtt.connect(MQTT_CLIENT_ID)) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void mqttTask(void *parameter) {
  mqtt.begin(MQTT_BROKER_ADRRESS, network);
  connectToMQTT();

  JoyMessage rcvMsg;
  char payload[10]; // Buffer สำหรับสร้างข้อความ Jxxx

  while (true) {
    if (!mqtt.connected()) connectToMQTT();
    mqtt.loop();

    // รอรับข้อมูลจาก Joystick Task
    if (xQueueReceive(mqttQueue, &rcvMsg, 0) == pdTRUE) {
      
      if (rcvMsg.type == 'J') {
        // สร้างข้อความเช่น "J90", "J180"
        sprintf(payload, "J%d", rcvMsg.value); 
        mqtt.publish(MQTT_TOPIC, payload);
        Serial.println(payload); // Debug ดูค่าที่ส่ง
      } 
      else if (rcvMsg.type == 'A') {
        // ส่งคำสั่ง AUTO
        mqtt.publish(MQTT_TOPIC, "AUTO");
        Serial.println("Sent: AUTO");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void joystickTask(void *parameter) {
  pinMode(JOY_SW_PIN, INPUT_PULLUP); // ตั้งค่าปุ่มกด

  int currentAngle = 90;
  int lastSentAngle = -1;
  int threshold = 3; // ต้องขยับเกิน 3 องศาถึงจะส่ง (กันค่าสั่น)

  while (true) {
    // 1. อ่านค่า Analog (0 - 4095)
    int rawX = analogRead(JOY_X_PIN);

    // 2. แปลงค่า (Map) จาก 0-4095 เป็น 0-360 องศา
    int mapAngle = map(rawX, 0, 4095, 0, 360);

    // 3. กรองค่าสั่น (Noise Filter)
    // ส่งค่าเมื่อมีการเปลี่ยนแปลงเกิน threshold หรือพึ่งเริ่มทำงาน
    if (abs(mapAngle - lastSentAngle) > threshold) {
      JoyMessage msg;
      msg.type = 'J';
      msg.value = mapAngle;
      
      xQueueSend(mqttQueue, &msg, 0);
      lastSentAngle = mapAngle;
    }

    // 4. (แถม) เช็คปุ่มกดเพื่อส่งคำสั่ง AUTO
    if (digitalRead(JOY_SW_PIN) == LOW) {
      JoyMessage msg;
      msg.type = 'A'; // Auto
      xQueueSend(mqttQueue, &msg, 0);
      vTaskDelay(500 / portTICK_PERIOD_MS); // กันกดเบิ้ล
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // อ่านค่าทุก 50ms
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(9600);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  // สร้าง Queue ให้รองรับ Struct ข้อมูล
  mqttQueue = xQueueCreate(10, sizeof(JoyMessage));

  xTaskCreate(mqttTask, "MQTT Task", 4096, NULL, 1, NULL);
  xTaskCreate(joystickTask, "Joy Task", 2048, NULL, 1, NULL);
}

void loop() {}