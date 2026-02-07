#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include <math.h>

// --- Config ---
#define JOY_X_PIN 34  
#define JOY_Y_PIN 35  
#define JOY_SW_PIN 23 

// ปรับความไวในการตรวจจับการขยับเพื่อเข้าโหมด Joy (ค่า 0-2048)
// ค่า 800 แปลว่าต้องโยกเกือบครึ่งถึงจะตัดเข้า Manual ป้องกันมือไปโดนนิดเดียวแล้วหลุด Auto
#define WAKE_UP_THRESHOLD 800 
#define DEADZONE 500

const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-joy-sender-smart"; 
const char MQTT_TOPIC[] = "esp32/command";

WiFiClient network;
MQTTClient mqtt(256);
QueueHandle_t mqttQueue;

struct JoyMessage { char type; int value; };

// *** 1. เริ่มต้นให้เป็น AUTO ทันที ***
bool inAutoMode = true; 
int lastSentAngle = -1;

// ตัวแปรสำหรับทำ Smoothing
float smoothX = 0;
float smoothY = 0;
float alpha = 0.1; 

void connectToMQTT() {
  while (!mqtt.connect(MQTT_CLIENT_ID)) vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void mqttTask(void *parameter) {
  mqtt.begin(MQTT_BROKER_ADRRESS, network);
  connectToMQTT();
  
  // *** ส่งสถานะ AUTO ครั้งแรกเมื่อเปิดเครื่อง ***
  mqtt.publish(MQTT_TOPIC, "AUTO");

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
  
  smoothX = analogRead(JOY_X_PIN);
  smoothY = analogRead(JOY_Y_PIN);

  while (true) {
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    // Smoothing Process
    smoothX = (smoothX * (1.0 - alpha)) + (rawX * alpha);
    smoothY = (smoothY * (1.0 - alpha)) + (rawY * alpha);

    // Map ค่าให้ 0 อยู่ตรงกลาง (-2048 ถึง 2048)
    int mapX = (int)smoothX - 2048; 
    int mapY = (int)smoothY - 2048;

    // คำนวณระยะห่างจากจุดศูนย์กลาง (Magnitude)
    double distance = sqrt((double)(mapX*mapX) + (double)(mapY*mapY));

    // --- 2. เช็คปุ่มกดเพื่อกลับเข้า Auto Mode ---
    if (digitalRead(JOY_SW_PIN) == LOW) {
      if (!inAutoMode) { // ถ้าไม่ได้เป็น Auto อยู่ ให้เปลี่ยนเป็น Auto
        inAutoMode = true;
        JoyMessage msg; msg.type = 'A';
        xQueueSend(mqttQueue, &msg, 0);
        Serial.println("Button Pressed -> Force AUTO");
        vTaskDelay(500 / portTICK_PERIOD_MS); // กันเบิ้ล
      }
    }

    // --- 3. เช็คการขยับจอยเพื่อปลุกเข้า Joy Mode ---
    // ถ้าอยู่ใน Auto และมีการโยกจอยแรงกว่า Threshold -> ยกเลิก Auto ทันที
    if (inAutoMode && distance > WAKE_UP_THRESHOLD) {
      inAutoMode = false;
      Serial.println("Movement Detected -> Switch to JOY");
      // ไม่ต้องส่งอะไรตอนนี้ เดี๋ยว loop ข้างล่างจะส่งค่า J เองทันที
    }

    // --- 4. การทำงานในโหมด Joystick ---
    if (!inAutoMode) {
        // ต้องขยับเกิน Deadzone ถึงจะส่งค่า (ป้องกันจอยหลวมแล้วค่าไหล)
        if (distance > DEADZONE) {
            
            double radian = atan2(mapY, mapX); 
            int angle = (radian * 180.0 / PI) + 180; // แปลงเป็น 0 - 360

            // Logic จำกัดองศาแค่ 270
            if (angle > 270) {
                angle = 270; 
            }

            // ส่งค่าเมื่อมีการเปลี่ยนแปลง
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
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  mqttQueue = xQueueCreate(10, sizeof(JoyMessage));
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(joystickTask, "Joy", 4096, NULL, 1, NULL);
}

void loop() {}