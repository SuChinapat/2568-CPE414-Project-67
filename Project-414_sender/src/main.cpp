#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include <math.h>

// *** เพิ่ม Library ป้องกันไฟตก ***
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- Config ---
#define JOY_X_PIN 34  
#define JOY_Y_PIN 35  
#define JOY_SW_PIN 23 

#define WAKE_UP_THRESHOLD 800 
#define DEADZONE 500

const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-joy-sender-180"; 
const char MQTT_TOPIC[] = "esp32/command";

WiFiClient network;
MQTTClient mqtt(256);
QueueHandle_t mqttQueue;

struct JoyMessage { char type; int value; };

bool inAutoMode = true; 
int lastSentAngle = -1;

float smoothX = 0;
float smoothY = 0;
float alpha = 0.1; 

void connectToMQTT() {
  Serial.print("Checking WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  Serial.println("\nWiFi Connected!");

  Serial.print("Connecting MQTT...");
  while (!mqtt.connect(MQTT_CLIENT_ID)) {
    Serial.print(".");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  Serial.println("\nMQTT Connected!");
}

void mqttTask(void *parameter) {
  mqtt.begin(MQTT_BROKER_ADRRESS, network);
  connectToMQTT();
  
  // ส่งค่าเริ่มต้น
  mqtt.publish(MQTT_TOPIC, "AUTO");
  Serial.println("Initial AUTO mode sent.");

  JoyMessage rcvMsg;
  char payload[10];
  while (true) {
    if (!mqtt.connected()) connectToMQTT();
    mqtt.loop();
    if (xQueueReceive(mqttQueue, &rcvMsg, 0) == pdTRUE) {
      if (rcvMsg.type == 'J') {
        sprintf(payload, "J%d", rcvMsg.value); 
        mqtt.publish(MQTT_TOPIC, payload);
        Serial.printf("Sent: %s\n", payload); // Debug print
      } else if (rcvMsg.type == 'A') {
        mqtt.publish(MQTT_TOPIC, "AUTO");
        Serial.println("Sent: AUTO"); // Debug print
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void joystickTask(void *parameter) {
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  
  smoothX = analogRead(JOY_X_PIN);
  smoothY = analogRead(JOY_Y_PIN);

  while (true) {
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    smoothX = (smoothX * (1.0 - alpha)) + (rawX * alpha);
    smoothY = (smoothY * (1.0 - alpha)) + (rawY * alpha);

    int mapX = (int)smoothX - 2048; 
    int mapY = (int)smoothY - 2048;

    double distance = sqrt((double)(mapX*mapX) + (double)(mapY*mapY));

    // ปุ่มกดเพื่อกลับเข้า Auto
    if (digitalRead(JOY_SW_PIN) == LOW) {
      if (!inAutoMode) { 
        inAutoMode = true;
        JoyMessage msg; msg.type = 'A';
        xQueueSend(mqttQueue, &msg, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS); 
      }
    }

    // ขยับจอยแรงๆ เพื่อเข้าโหมด Joy
    if (inAutoMode && distance > WAKE_UP_THRESHOLD) {
      inAutoMode = false;
      Serial.println("Joy Moved -> Manual Mode");
    }

    if (!inAutoMode) {
        if (distance > DEADZONE) {
            
            double radian = atan2(mapY, mapX); 
            int angle = (radian * 180.0 / PI) + 180; 

            if (angle > 180) {
                angle = 180; 
            }

            if (abs(angle - lastSentAngle) > 2) {
                JoyMessage msg; 
                msg.type = 'J'; 
                msg.value = angle;
                xQueueSend(mqttQueue, &msg, 0);
                lastSentAngle = angle;
            }
        }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  // 1. ปิด Brownout Detector ทันทีที่เริ่ม
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  
  // 2. รอ 1 วินาที เพื่อให้ Serial Monitor พร้อม และไฟนิ่ง
  delay(1000); 
  Serial.println("\n--- Sender Starting ---");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected (Setup Phase)");

  mqttQueue = xQueueCreate(10, sizeof(JoyMessage));
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(joystickTask, "Joy", 4096, NULL, 1, NULL);
}

void loop() {}