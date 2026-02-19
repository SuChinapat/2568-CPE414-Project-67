#include <Arduino.h>

#include <WiFi.h>

#include <esp_now.h>

#include <math.h>

// Brownout protection

#include "soc/soc.h"

#include "soc/rtc_cntl_reg.h"

// --- Config ---

uint8_t broadcastAddress[] = {0xD8, 0x13, 0x2A, 0x7F, 0x92, 0x5C};

const char WIFI_SSID[] = "116_2.4G";

const char WIFI_PASSWORD[] = "0816978323";

#define JOY_X_PIN 34

#define JOY_Y_PIN 35

#define JOY_SW_PIN 23

// Adjusted Thresholds for 3.3V

#define WAKE_UP_THRESHOLD 500

#define DEADZONE 150

typedef struct struct_message

{

  char type;

  int value;

} struct_message;

esp_now_peer_info_t peerInfo;

QueueHandle_t sendQueue;

bool inAutoMode = true;

int centerX, centerY;

// Filter Settings

#define FILTER_SIZE 15

int readingsX[FILTER_SIZE];

int readingsY[FILTER_SIZE];

int readIndex = 0;

long totalX = 0;

long totalY = 0;

// --- Task 1: ESP-NOW Transmission ---

void espNowTask(void *parameter)

{

  if (esp_now_init() != ESP_OK)

    vTaskDelete(NULL);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  peerInfo.channel = 0;

  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)

    vTaskDelete(NULL);

  struct_message msgToSend;

  while (true)

  {

    if (xQueueReceive(sendQueue, &msgToSend, portMAX_DELAY) == pdTRUE)

    {

      esp_now_send(broadcastAddress, (uint8_t *)&msgToSend, sizeof(msgToSend));
    }
  }
}

// --- Smoothing Function ---

void readJoystickSmooth(int *outX, int *outY)

{

  totalX -= readingsX[readIndex];

  totalY -= readingsY[readIndex];

  readingsX[readIndex] = analogRead(JOY_X_PIN);

  readingsY[readIndex] = analogRead(JOY_Y_PIN);

  totalX += readingsX[readIndex];

  totalY += readingsY[readIndex];

  readIndex = (readIndex + 1) % FILTER_SIZE;

  *outX = totalX / FILTER_SIZE;

  *outY = totalY / FILTER_SIZE;
}

// --- Task 2: 180-Degree Joystick Logic ---

void joystickTask(void *parameter)

{

  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  // Pre-fill filter with center values

  for (int i = 0; i < FILTER_SIZE; i++)

  {

    readingsX[i] = centerX;

    readingsY[i] = centerY;
  }

  totalX = (long)centerX * FILTER_SIZE;

  totalY = (long)centerY * FILTER_SIZE;

  while (true)

  {

    int smoothX, smoothY;

    readJoystickSmooth(&smoothX, &smoothY);

    // X and Y mapping

    int mapX = centerX - smoothX;

    int mapY = centerY - smoothY;

    double distance = sqrt((double)mapX * mapX + (double)mapY * mapY);

    // 1. Button Logic (Mode Switch)

    if (digitalRead(JOY_SW_PIN) == LOW)

    {

      if (!inAutoMode)

      {

        inAutoMode = true;

        struct_message msg = {'A', 0};

        xQueueSend(sendQueue, &msg, 0);

        Serial.println(">> SYSTEM: AUTO MODE");

        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    // 2. Wake-up Logic

    if (inAutoMode && distance > WAKE_UP_THRESHOLD)

    {

      inAutoMode = false;

      Serial.println(">> SYSTEM: MANUAL MODE");
    }

    // 3. 180-Degree Calculation

    if (!inAutoMode && distance > DEADZONE)

    {

      // Use atan2 to get the full angle

      double radian = atan2(mapY, mapX);

      int angle = (int)(radian * 180.0 / PI);

      // Map the joystick sweep to a 0-180 servo range

      // We clamp it here so only the upper arc works.

      // make sure that the angle is between 0 and 180

      if (mapY < 0)
      {

        if (mapX > 0)

          angle = 0;

        if (mapX < 0)

          angle = 180;
      }

      // Invert angle range: 0-180 becomes 180-0
      angle = 180 - angle;

      struct_message msg = {'J', angle};

      if (xQueueSend(sendQueue, &msg, 0) == pdTRUE)
      {
        Serial.printf("POS -> X:%d Y:%d | SERVO ANGLE:%d\n", mapX, mapY, angle);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void setup()

{

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  // Critical for 3.3V Joystick Power

  analogSetAttenuation(ADC_11db);

  delay(1000);

  centerX = analogRead(JOY_X_PIN);

  centerY = analogRead(JOY_Y_PIN);

  Serial.printf("Calibration: X=%d, Y=%d\n", centerX, centerY);

  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retry = 0;

  while (WiFi.status() != WL_CONNECTED && retry < 15)

  {

    delay(500);

    Serial.print(".");

    retry++;
  }

  Serial.println("\nReady!");

  sendQueue = xQueueCreate(20, sizeof(struct_message));

  // Use Core 0 for sensors and Core 1 for Radio/WiFi

  xTaskCreatePinnedToCore(espNowTask, "ESP-NOW", 4096, NULL, 2, NULL, 1);

  xTaskCreatePinnedToCore(joystickTask, "Joystick", 4096, NULL, 1, NULL, 0);
}

void loop()

{
  // FreeRTOS takes care of the rest
}
