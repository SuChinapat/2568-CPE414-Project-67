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
#define DEADZONE 250

const char WIFI_SSID[] = "Mi 10T";
const char WIFI_PASSWORD[] = "0123456789";
const char MQTT_BROKER_ADRRESS[] = "broker.hivemq.com";
const char MQTT_CLIENT_ID[] = "esp32-radar-180-fast-87342";
const char MQTT_TOPIC[] = "esp32/radar_87342/control";


WiFiClient network;
MQTTClient mqtt(256);
QueueHandle_t mqttQueue;

struct JoyMessage { char type; int value; };

bool inAutoMode = true; 
int lastSentAngle = -1;

int centerX;
int centerY;

void connectToMQTT() {

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Waiting WiFi...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  mqtt.begin(MQTT_BROKER_ADRRESS, 1883, network);


  while (!mqtt.connect(MQTT_CLIENT_ID)) {
    Serial.println("MQTT connect failed, retrying...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  Serial.println("MQTT Connected!");
}


void mqttTask(void *parameter) {
  mqtt.begin(MQTT_BROKER_ADRRESS, 1883, network);

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
  
  int rawX = analogRead(JOY_X_PIN);
  int rawY = analogRead(JOY_Y_PIN);

  while (true) {
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    Serial.print("rawX: "); Serial.print(rawX);
    Serial.print(" rawY: "); Serial.println(rawY);


    int mapX = rawX - centerX;
    int mapY = rawY - centerY;

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

    if (inAutoMode && distance > WAKE_UP_THRESHOLD) {
      inAutoMode = false;
      Serial.println(distance);
      Serial.println("Joy Moved -> Manual Mode");
    }

    if (!inAutoMode) {
        if (distance > DEADZONE) {
            
            double radian = atan2(mapY, mapX);
            int angle = radian * (180.0 / PI);
            // Serial.println(angle);

            // ทำให้เป็น 0-360
            if (angle < 0) angle += 360;

            // บีบเหลือ 0-180 แบบ mirror
            if (angle > 180) {
                angle = 360 - angle;
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
  analogSetAttenuation(ADC_11db);

  // 1. ปิด Brownout Detector ทันทีที่เริ่ม
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(921600);
  
  // 2. รอ 1 วินาที เพื่อให้ Serial Monitor พร้อม และไฟนิ่ง
  delay(1000); 
  centerX = analogRead(JOY_X_PIN);
  centerY = analogRead(JOY_Y_PIN);

  Serial.print("CenterX: "); Serial.println(centerX);
  Serial.print("CenterY: "); Serial.println(centerY);
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
