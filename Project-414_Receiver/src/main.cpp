#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include <WebServer.h>
#include <ESP32Servo.h> 
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "DFRobotDFPlayerMini.h" // ต้องติดตั้ง Library นี้

// --- Pin Config ---
#define TRIG_PIN 12
#define ECHO_PIN 14
#define SERVO_PIN 13

// --- DFPlayer Pin Config (ใช้ Hardware Serial 1 หรือ 2) ---
#define DFPLAYER_RX_PIN 16 // ต่อกับ TX ของ DFPlayer
#define DFPLAYER_TX_PIN 17 // ต่อกับ RX ของ DFPlayer

// --- Wi-Fi & MQTT ---
const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-radar-receiver-270"; 
const char MQTT_TOPIC[] = "esp32/command";

WiFiClient network;
MQTTClient mqtt(512); 
WebServer server(80);
Servo myServo;

// --- Audio Objects ---
HardwareSerial mySerial(1); // ใช้ UART 1
DFRobotDFPlayerMini myDFPlayer;

// --- Global Variables ---
volatile int currentAngle = 135; 
volatile int currentDistance = 0;
volatile bool isSystemArmed = true; 
volatile bool isJoyMode = false;
volatile int joyTargetAngle = 135;

// สถานะการเล่นเสียง (เพื่อป้องกันการสั่ง Play ซ้ำๆ จนเสียงกระตุก)
bool isPlaying = false;

// --- HTML Real-Time Update (คงเดิม) ---
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
  <h1>REAL-TIME RADAR (270&deg;)</h1>
  <canvas id="radar" width="400" height="400"></canvas>
  <div id="info">Waiting...</div>
<script>
  var canvas = document.getElementById("radar");
  var ctx = canvas.getContext("2d");
  var width = canvas.width; var height = canvas.height;
  var cx = width/2; var cy = height/2; var radius = (width/2)-10;

  function drawRadar(angle, dist) {
    ctx.fillStyle = "rgba(0,0,0,0.1)"; 
    ctx.fillRect(0,0,width,height);

    ctx.strokeStyle = "#004400"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, radius, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.66, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.33, 0, 2*Math.PI); ctx.stroke();
    
    var finalRad = (angle - 225) * (Math.PI / 180); 
    
    ctx.strokeStyle = "#00FF00"; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(finalRad) * radius, cy + Math.sin(finalRad) * radius);
    ctx.stroke();

    if(dist > 0 && dist < 40) {
      var pixDist = dist * (radius / 40.0);
      var ox = cx + Math.cos(finalRad) * pixDist;
      var oy = cy + Math.sin(finalRad) * pixDist;
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

// --- Measure Function ---
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); 
  long dist = duration * 0.034 / 2;
  if (dist == 0 || dist > 200) dist = 200; 
  return dist;
}

// --- Task 1: Radar Logic ---
void radarTask(void *parameter) {
  myServo.attach(SERVO_PIN, 500, 2500); 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  int sweepDir = 1;

  while (true) {
    if (isJoyMode) {
      myServo.write(joyTargetAngle);
      currentAngle = joyTargetAngle; 
      currentDistance = measureDistance();
      vTaskDelay(10 / portTICK_PERIOD_MS); 
    } 
    else {
      myServo.write(currentAngle);
      currentDistance = measureDistance();
      
      currentAngle += (sweepDir * 2); 
      
      if (currentAngle >= 270) { currentAngle = 270; sweepDir = -1; } 
      if (currentAngle <= 0) { currentAngle = 0; sweepDir = 1; }
      
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }
  }
}

// --- Task 2: DFPlayer Control (Audio) ---
// เปลี่ยนจาก buzzerTask เป็น audioTask
void audioTask(void *parameter) {
  // เริ่มต้น Serial สำหรับ DFPlayer (RX=16, TX=17)
  mySerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  
  // รอให้ module พร้อม
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  if (!myDFPlayer.begin(mySerial, /*isACK = */true, /*doReset = */true)) {
    Serial.println(F("Unable to begin DFPlayer: Please check connection!"));
    // ถ้าต่อไม่ติด อาจจะเลือกที่จะทำงานต่อโดยไม่มีเสียง หรือหยุด loop ก็ได้
  } else {
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.volume(20);  // ตั้งความดัง 0-30
  }

  while (true) {
    // เงื่อนไข: ระยะน้อยกว่า 40 ซม. และระบบ Armed อยู่
    if (currentDistance > 0 && currentDistance < 40 && isSystemArmed) {
      if (!isPlaying) {
        // ถ้ายังไม่ได้เล่นอยู่ ให้สั่งเล่น
        Serial.println("Object Detected! Playing Sound...");
        // myDFPlayer.play(1); // เล่นเพลงที่ 1 ครั้งเดียว
        myDFPlayer.loop(1);    // หรือ เล่นเพลงที่ 1 วนลูปไปเรื่อยๆ จนกว่าจะสั่งหยุด
        isPlaying = true;
      }
    } else {
      // ถ้าไม่มีวัตถุ หรือระบบ Disarm
      if (isPlaying) {
        Serial.println("Clear! Stopping Sound.");
        myDFPlayer.pause(); // หรือ myDFPlayer.stop();
        isPlaying = false;
      }
    }
    
    // หน่วงเวลาเล็กน้อยเพื่อไม่ให้ loop เร็วเกินไปจนกินทรัพยากร
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- MQTT Callback ---
void messageReceived(String &topic, String &payload) {
  if (payload.startsWith("J")) {
    isJoyMode = true;
    int angle = payload.substring(1).toInt();
    if (angle < 0) angle = 0;
    if (angle > 270) angle = 270; 
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
    vTaskDelay(5 / portTICK_PERIOD_MS);
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
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("Web Server IP: ");
  Serial.println(WiFi.localIP());

  // สร้าง Tasks
  xTaskCreate(radarTask, "Radar", 2048, NULL, 1, NULL);
  // เพิ่ม Stack Size ให้ audioTask หน่อยเพราะ Library นี้ใช้ Serial buffer
  xTaskCreate(audioTask, "Audio", 4096, NULL, 1, NULL); 
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(serverTask, "Web", 4096, NULL, 1, NULL);
}

void loop() {}