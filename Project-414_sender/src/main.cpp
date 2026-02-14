#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// *** เพิ่ม Library ป้องกันไฟตก ***
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- Config ---
// *** 🔴 ใส่ MAC Address ของตัวรับ (Receiver) ที่นี่ 🔴 ***
uint8_t broadcastAddress[] = {0x30, 0xC9, 0x22, 0x33, 0x19, 0x20}; 

// WiFi ชื่อเดียวกับตัวรับ
const char WIFI_SSID[] = "Mi 10T";
const char WIFI_PASSWORD[] = "0123456789";

#define JOY_X_PIN 34
#define JOY_Y_PIN 35
#define JOY_SW_PIN 23

#define WAKE_UP_THRESHOLD 800
#define DEADZONE 250

// โครงสร้างข้อมูล
typedef struct struct_message {
  char type;
  int value;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;
QueueHandle_t sendQueue;

bool inAutoMode = true;
int lastSentAngle = -1;
int centerX, centerY;

// *** เพิ่มตัวแปรสำหรับ Filter ***
#define FILTER_SIZE 10 // อ่าน 10 ครั้งแล้วหาค่าเฉลี่ย
int readingsX[FILTER_SIZE];
int readingsY[FILTER_SIZE];
int readIndex = 0;
long totalX = 0;
long totalY = 0;

// --- Task 1: ส่งข้อมูล ESP-NOW ---
void espNowTask(void *parameter) {
  if (esp_now_init() != ESP_OK) vTaskDelete(NULL);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) vTaskDelete(NULL);

  myData.type = 'A';
  myData.value = 0;
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  struct_message msgToSend;
  while (true) {
    if (xQueueReceive(sendQueue, &msgToSend, portMAX_DELAY) == pdTRUE) {
        esp_now_send(broadcastAddress, (uint8_t *) &msgToSend, sizeof(msgToSend));
    }
  }
}

// --- ฟังก์ชันอ่านจอยแบบนิ่งๆ (Smooth Read) ---
void readJoystickSmooth(int *outX, int *outY) {
  // ลบค่าเก่าออก
  totalX = totalX - readingsX[readIndex];
  totalY = totalY - readingsY[readIndex];
  
  // อ่านค่าใหม่
  readingsX[readIndex] = analogRead(JOY_X_PIN);
  readingsY[readIndex] = analogRead(JOY_Y_PIN);
  
  // รวมค่าใหม่
  totalX = totalX + readingsX[readIndex];
  totalY = totalY + readingsY[readIndex];
  
  // เลื่อน Index
  readIndex = (readIndex + 1);
  if (readIndex >= FILTER_SIZE) readIndex = 0;

  // คืนค่าเฉลี่ย
  *outX = totalX / FILTER_SIZE;
  *outY = totalY / FILTER_SIZE;
}

// --- Task 2: อ่าน Joystick (Uncomment และเพิ่ม Filter) ---
void joystickTask(void *parameter) {
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  
  // Init Filter Array
  for (int i = 0; i < FILTER_SIZE; i++) {
    readingsX[i] = centerX;
    readingsY[i] = centerY;
    totalX += centerX;
    totalY += centerY;
  }

  while (true) {
    int smoothX, smoothY;
    readJoystickSmooth(&smoothX, &smoothY); // อ่านแบบนิ่งๆ

    int mapX = smoothX - centerX;
    int mapY = smoothY - centerY;

    double distance = sqrt((double)(mapX*mapX) + (double)(mapY*mapY));

    // 1. ปุ่มกด -> กลับ Auto
    if (digitalRead(JOY_SW_PIN) == LOW) {
      if (!inAutoMode) {
        inAutoMode = true;
        struct_message msg; msg.type = 'A'; msg.value = 0;
        xQueueSend(sendQueue, &msg, 0);
        Serial.println("Sent: AUTO");
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
    }

    // 2. ขยับจอย -> Manual (Uncommented)
    if (inAutoMode && distance > WAKE_UP_THRESHOLD) {
      inAutoMode = false;
      Serial.println("Manual Mode Activated!");
    }

    // 3. ส่งค่า (Manual Mode) (Uncommented)
    if (!inAutoMode) {
        if (distance > DEADZONE) {
            double radian = atan2(mapY, mapX);
            int angle = radian * (180.0 / PI);
            if (angle < 0) angle += 360;
            if (angle > 180) angle = 360 - angle;

            // *** เพิ่ม Hysteresis: ต้องเปลี่ยนเกิน 2 องศาค่อยส่ง ***
            if (abs(angle - lastSentAngle) > 2) { 
                struct_message msg;
                msg.type = 'J';
                msg.value = angle;
                xQueueSend(sendQueue, &msg, 0);
                lastSentAngle = angle;
            }
        }
    }
    
    // อ่านถี่ๆ เพื่อให้ค่าเฉลี่ยแม่นยำ (5ms)
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(921600);
  
  delay(1000);
  centerX = analogRead(JOY_X_PIN);
  centerY = analogRead(JOY_Y_PIN);

  // เช็คสายขาด
  if(centerY < 100) Serial.println("WARNING: JOYSTICK DISCONNECTED!");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nReady!");

  sendQueue = xQueueCreate(20, sizeof(struct_message));
  xTaskCreate(espNowTask, "ESP-NOW", 4096, NULL, 1, NULL);
  xTaskCreate(joystickTask, "Joy", 4096, NULL, 1, NULL);
}

void loop() {}
