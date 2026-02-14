#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "DFRobotDFPlayerMini.h"

// ---------------- PIN ----------------
#define TRIG_PIN 27
#define ECHO_PIN 14
#define SERVO_PIN 13
#define DFPLAYER_RX_PIN 16
#define DFPLAYER_TX_PIN 17

// ---------------- WIFI ----------------
const char WIFI_SSID[] = "Mi 10T";
const char WIFI_PASSWORD[] = "0123456789";

// ---------------- JOY CONFIG ----------------
#define DEADZONE 2
#define JOY_TIMEOUT 800 // เวลา Timeout ถ้าไม่ขยับจอยจะกลับ Auto

// ---------------- STRUCT ----------------
typedef struct struct_message {
  char type;
  int value;
} struct_message;

struct_message myData;

// ---------------- OBJECTS ----------------
WebServer server(80);
Servo myServo;
HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

// ---------------- GLOBAL ----------------
volatile int currentAngle = 90;
volatile int currentDistance = 0;
volatile bool isJoyMode = false;
volatile int joyTargetAngle = 90;
volatile unsigned long lastJoyMoveTime = 0;
volatile bool isSystemArmed = true;

float smoothAngle = 90.0;
bool isPlaying = false;

// ---------------- ESP-NOW CALLBACK ----------------
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // กรณี 1: ขยับจอย (Joystick Move)
  if (myData.type == 'J') {
    int raw = myData.value;

    if (raw > 180) raw = 180;
    if (raw < 0) raw = 0;

    // อัปเดตเป้าหมาย
    joyTargetAngle = raw;
    isJoyMode = true;
    lastJoyMoveTime = millis();
  }
  // กรณี 2: กดปุ่มหัวจอย (Button Press -> Auto)
  else if (myData.type == 'A') {
    isJoyMode = false; // บังคับกลับ Auto ทันที
  }
}

// ---------------- DISTANCE ----------------
long measureDistanceFast() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 15000);
  long dist = duration * 0.034 / 2;

  if (dist > 1 && dist < 200) return dist;
  return currentDistance;
}

// ---------------- RADAR TASK ----------------
void radarTask(void *parameter) {
  myServo.attach(SERVO_PIN, 500, 2400);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  int sweepDir = 1;
  int autoAngle = 90;
  unsigned long lastScan = 0;

  while (true) {
    // 🔁 เช็ค Timeout: ถ้าไม่ขยับจอยนานเกินกำหนด ให้กลับ Auto
    if (isJoyMode && millis() - lastJoyMoveTime > JOY_TIMEOUT) {
      isJoyMode = false;
    }

    if (isJoyMode) {
      // 🎮 MANUAL MODE (แบบมี Smooth)
      // ถ้าอยากให้ไวสุดๆ ให้แก้บรรทัดนี้เป็น: smoothAngle = joyTargetAngle;
      smoothAngle = smoothAngle + (joyTargetAngle - smoothAngle) * 0.25;

      currentAngle = (int)smoothAngle;
      myServo.write(currentAngle);

      // วัดระยะ (ไม่ต้องถี่มาก เดี๋ยว Servo กระตุก)
      if (millis() - lastScan > 60) {
        currentDistance = measureDistanceFast();
        lastScan = millis();
      }
      vTaskDelay(15 / portTICK_PERIOD_MS);

    } else {
      // 🔄 AUTO MODE
      autoAngle += sweepDir * 2;
      if (autoAngle >= 180) { autoAngle = 180; sweepDir = -1; }
      if (autoAngle <= 0)   { autoAngle = 0;   sweepDir = 1; }

      // Smooth Auto Movement
      smoothAngle = smoothAngle + (autoAngle - smoothAngle) * 0.2;
      currentAngle = (int)smoothAngle;
      myServo.write(currentAngle);

      currentDistance = measureDistanceFast();
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }
  }
}

// ---------------- AUDIO TASK ----------------
void audioTask(void *parameter) {
  mySerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);

  if (myDFPlayer.begin(mySerial)) {
    myDFPlayer.volume(15);
  }

  while (true) {
    bool detect = (currentDistance > 1 && currentDistance < 40 && isSystemArmed);

    if (detect && !isPlaying) {
      myDFPlayer.play(1);
      isPlaying = true;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      isPlaying = false;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ---------------- WEB ----------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>ESP32 Radar</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body{background:#000;color:#0f0;font-family:Courier;text-align:center;display:flex;flex-direction:column;align-items:center;}
canvas{background:#111;width:100%;max-width:400px;border-radius:10px;box-shadow:0 0 20px #0f0;}
#info{font-size:1.2rem;margin-top:20px;border:1px solid #0f0;padding:10px;width:100%;max-width:380px;}
</style></head>
<body>
<h1>RADAR (180&deg;)</h1>
<canvas id="radar" width="400" height="250"></canvas>
<div id="info">Waiting...</div>
<script>
var ctx = document.getElementById("radar").getContext("2d");
var cx=200, cy=230, r=190;
function draw(angle, dist){
  ctx.fillStyle="rgba(0,0,0,0.1)"; ctx.fillRect(0,0,400,250);
  
  // Grid
  ctx.strokeStyle="#004400"; ctx.lineWidth=2;
  ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, 2*Math.PI); ctx.stroke();
  
  // Needle (Mirror Logic)
  var rad=Math.PI+((180-angle)*Math.PI/180);
  ctx.strokeStyle="#0f0"; ctx.lineWidth=3;
  ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(cx+Math.cos(rad)*r,cy+Math.sin(rad)*r); ctx.stroke();
  
  // Object
  if(dist>0 && dist<40){
    var pd = dist*(r/40.0);
    ctx.fillStyle="red"; ctx.beginPath(); ctx.arc(cx+Math.cos(rad)*pd, cy+Math.sin(rad)*pd, 10, 0, 2*Math.PI); ctx.fill();
  }
}
setInterval(()=>{
fetch("/data").then(r=>r.json()).then(d=>{
document.getElementById("info").innerHTML="MODE: "+(d.mode?"JOY":"AUTO")+"<br>ANG: "+d.angle+" | DIST: "+d.dist;
draw(d.angle,d.dist);
});
},50);
</script></body></html>
)rawliteral";

void serverTask(void *parameter) {
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", index_html); });
  server.on("/data", HTTP_GET, []() {
    String json = "{\"angle\":" + String(currentAngle) +
                  ",\"dist\":" + String(currentDistance) +
                  ",\"mode\":" + String(isJoyMode) + "}";
    server.send(200, "application/json", json);
  });
  server.begin();
  while (true) {
    server.handleClient();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// ---------------- SETUP ----------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  // Hybrid Mode: AP+STA
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());
  Serial.println("Channel: " + String(WiFi.channel()));

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Error");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  xTaskCreate(radarTask, "Radar", 4096, NULL, 1, NULL);
  xTaskCreate(audioTask, "Audio", 4096, NULL, 1, NULL);
  xTaskCreate(serverTask,"Web",   4096, NULL, 1, NULL);
}

void loop() {}
