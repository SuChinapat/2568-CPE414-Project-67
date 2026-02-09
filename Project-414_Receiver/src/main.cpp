#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include <WebServer.h>
#include <ESP32Servo.h> 
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "DFRobotDFPlayerMini.h" 

// --- Pin Config ---
#define TRIG_PIN 27
#define ECHO_PIN 14
#define SERVO_PIN 13
#define DFPLAYER_RX_PIN 16 
#define DFPLAYER_TX_PIN 17 

// --- Wi-Fi & MQTT ---
const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-radar-receiver-180"; 
const char MQTT_TOPIC[] = "esp32/command";

WiFiClient network;
MQTTClient mqtt(512); 
WebServer server(80);
Servo myServo;

HardwareSerial mySerial(1); 
DFRobotDFPlayerMini myDFPlayer;

// --- Global Variables ---
volatile int currentAngle = 90; 
volatile int currentDistance = 0;
volatile bool isSystemArmed = true; 
volatile bool isJoyMode = false;
volatile int joyTargetAngle = 90;
bool isPlaying = false;

// --- HTML (เหมือนเดิม ย่อไว้เพื่อความกระชับ) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Radar Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { background-color: #000; color: #0f0; font-family: courier; text-align: center; margin: 0; }
    canvas { background-color: #111; border-radius: 50%; box-shadow: 0 0 20px #0f0; margin-top: 20px;}
    #info { font-size: 20px; margin-top: 10px; font-weight: bold;}
  </style>
</head>
<body>
  <h1>REAL-TIME RADAR (180&deg;)</h1>
  <canvas id="radar" width="400" height="250"></canvas>
  <div id="info">Waiting...</div>
<script>
  var canvas = document.getElementById("radar");
  var ctx = canvas.getContext("2d");
  var width = canvas.width; var height = canvas.height;
  var cx = width/2; var cy = height - 20; 
  var radius = (width/2)-10;

  function drawRadar(angle, dist) {
    ctx.fillStyle = "rgba(0,0,0,0.1)"; 
    ctx.fillRect(0,0,width,height);

    ctx.strokeStyle = "#004400"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, radius, Math.PI, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.66, Math.PI, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.33, Math.PI, 2*Math.PI); ctx.stroke();
    
    var rad = Math.PI + (angle * Math.PI / 180);

    ctx.strokeStyle = "#00FF00"; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(rad) * radius, cy + Math.sin(rad) * radius);
    ctx.stroke();

    if(dist > 0 && dist < 40) {
      var pixDist = dist * (radius / 40.0);
      var ox = cx + Math.cos(rad) * pixDist;
      var oy = cy + Math.sin(rad) * pixDist;
      ctx.fillStyle = "red"; ctx.beginPath(); ctx.arc(ox, oy, 8, 0, 2*Math.PI); ctx.fill();
    }
  }

  setInterval(function() {
    fetch("/data").then(r => r.json()).then(d => {
      document.getElementById("info").innerHTML = "Mode: " + (d.mode ? "JOY" : "AUTO") + " | Angle: " + d.angle + "&deg;";
      drawRadar(d.angle, d.dist);
    });
  }, 40); 
</script>
</body>
</html>
)rawliteral";

long measureDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); 
  long dist = duration * 0.034 / 2;
  if (dist == 0 || dist > 200) dist = 200; 
  return dist;
}

void radarTask(void *parameter) {
  myServo.attach(SERVO_PIN, 500, 2400); 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  int sweepDir = 1;

  while (true) {
    if (isJoyMode) {
      myServo.write(joyTargetAngle);
      currentAngle = joyTargetAngle; 
      currentDistance = measureDistance();
      vTaskDelay(15 / portTICK_PERIOD_MS); 
    } 
    else {
      myServo.write(currentAngle);
      currentDistance = measureDistance();
      currentAngle += (sweepDir * 2); 
      if (currentAngle >= 180) { currentAngle = 180; sweepDir = -1; } 
      if (currentAngle <= 0)   { currentAngle = 0;   sweepDir = 1; }
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }
  }
}

void audioTask(void *parameter) {
  // เริ่มต้น Serial ให้ DFPlayer
  mySerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // รอให้ DFPlayer ตื่นเต็มที่

  // สั่งเชื่อมต่อ (ปิด ACK เพื่อความลื่นไหล)
  if (!myDFPlayer.begin(mySerial, /*isACK=*/false, /*doReset=*/true)) {
    Serial.println(F("DFPlayer Error: Check SD Card / Wiring"));
  } else {
    Serial.println(F("DFPlayer Online"));
    myDFPlayer.volume(15); // ตั้งความดัง 0-30
  }

  unsigned long lastTriggerTime = 0;
  const int keepPlayingDuration = 2000; // ถ้าเจอวัตถุ จะเล่นต่อเนื่องอย่างน้อย 2 วินาที

  while (true) {
    // 1. เช็คว่าเจอวัตถุไหม (ตัดค่า 0 ที่เกิดจาก Error ออก)
    bool objectDetected = (currentDistance > 1 && currentDistance < 40 && isSystemArmed);

    if (objectDetected) {
      lastTriggerTime = millis(); // จำเวลาล่าสุดที่เจอ
      
      if (!isPlaying) {
        // ถ้ายังไม่เล่น ให้เริ่มเล่น
        myDFPlayer.loop(1); // เล่นไฟล์ 0001.mp3 วนไป
        isPlaying = true;
        Serial.println(">>> START PLAYING");
      }
    } 
    else {
      // 2. ถ้าไม่เจอวัตถุ... ให้รอดูเวลาก่อน อย่าเพิ่งรีบปิด
      // ถ้าเวลาผ่านไปเกิน 2 วินาทีแล้ว ค่อยสั่งหยุด
      if (isPlaying && (millis() - lastTriggerTime > keepPlayingDuration)) {
        myDFPlayer.pause();
        isPlaying = false;
        Serial.println("||| STOP PLAYING (Timeout)");
      }
    }

    // หน่วงเวลา Loop ไม่ให้ทำงานถี่เกินไป (ลดภาระ CPU)
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void messageReceived(String &topic, String &payload) {
  if (payload.startsWith("J")) {
    isJoyMode = true;
    int angle = payload.substring(1).toInt();
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180; 
    joyTargetAngle = angle;
  }
  else if (payload == "AUTO") isJoyMode = false;
  else if (payload == "1") isSystemArmed = true;
  else if (payload == "0") isSystemArmed = false;
}

void connectToMQTT() {
  while (!mqtt.connect(MQTT_CLIENT_ID)) vTaskDelay(1000 / portTICK_PERIOD_MS);
  mqtt.subscribe(MQTT_TOPIC);
}

void mqttTask(void *parameter) {
  mqtt.begin(MQTT_BROKER_ADRRESS, network);
  mqtt.onMessage(messageReceived);
  connectToMQTT();
  while (true) {
    if (!mqtt.connected()) connectToMQTT();
    mqtt.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void serverTask(void *parameter) {
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", index_html); });
  server.on("/data", HTTP_GET, []() {
    String json = "{\"angle\":" + String(currentAngle) + 
                  ", \"dist\":" + String(currentDistance) + 
                  ", \"mode\":" + String(isJoyMode) + "}";
    server.send(200, "application/json", json);
  });
  server.begin();
  while (true) {
    server.handleClient();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200); 

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.println(WiFi.localIP());

  // Task Creation
  xTaskCreate(radarTask, "Radar", 2048, NULL, 1, NULL);
  // เพิ่ม Stack Size เผื่อไว้หน่อย
  xTaskCreate(audioTask, "Audio", 4096, NULL, 1, NULL); 
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(serverTask, "Web", 4096, NULL, 1, NULL);
}

void loop() {}