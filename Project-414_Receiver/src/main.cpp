#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
#include <WebServer.h>
#include <ESP32Servo.h> 
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "pitches.h" // อย่าลืมไฟล์นี้นะครับ

// --- Pin Config ---
#define BUZZER_PIN 5
#define TRIG_PIN 12
#define ECHO_PIN 14
#define SERVO_PIN 13

// --- Wi-Fi & MQTT ---
const char WIFI_SSID[] = "Pakorn 2.4G";
const char WIFI_PASSWORD[] = "0819249457";
const char MQTT_BROKER_ADRRESS[] = "test.mosquitto.org";
const char MQTT_CLIENT_ID[] = "esp32-radar-receiver-270"; 
const char MQTT_TOPIC[] = "esp32/command";

WiFiClient network;
MQTTClient mqtt(512); // เพิ่ม Buffer เผื่อ JSON ยาว
WebServer server(80);
Servo myServo;

// --- Global Variables ---
volatile int currentAngle = 135; // เริ่มที่ตรงกลางของ 270 (ประมาณ 135)
volatile int currentDistance = 0;
volatile bool isSystemArmed = true; 
volatile bool isJoyMode = false;
volatile int joyTargetAngle = 135;

// --- HTML Real-Time Update ---
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
    // 1. Fade Effect
    ctx.fillStyle = "rgba(0,0,0,0.1)"; 
    ctx.fillRect(0,0,width,height);

    // 2. Draw Grid
    ctx.strokeStyle = "#004400"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, radius, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.66, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.33, 0, 2*Math.PI); ctx.stroke();
    
    // 3. Draw Line (ปรับการแสดงผลองศาให้สวยงาม)
    // หมายเหตุ: การแสดงผลขึ้นอยู่กับการติดตั้ง Servo ว่า 0 อยู่ทิศไหน
    // สูตรนี้: (angle - 90) คือการหมุนแกนวาดรูป
    var rad = (angle - 135) * (Math.PI / 180); // ปรับ Offset ให้ 135 อยู่ตรงกลางบน (ถ้าต้องการ)
    // หรือใช้สูตรเดิม: var rad = (angle - 90) * (Math.PI / 180);
    
    // ใช้สูตรเดิมตามที่คุณเคยใช้เพื่อให้ทิศไม่เพี้ยนไปจากความเคยชิน
    var drawRad = (angle * Math.PI / 180) - (Math.PI / 2); // -90 องศาเพื่อให้ 0 เริ่มที่ 12 นาฬิกา หรือปรับตามจริง

    // เอาแบบ Basic สุดที่ตรงกับค่า Angle
    // var finalRad = (angle - 90) * (Math.PI / 180);
    var finalRad = (angle - 225) * (Math.PI / 180); // ปรับ Offset ให้ตรงกับการติดตั้งจริง
    
    ctx.strokeStyle = "#00FF00"; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(finalRad) * radius, cy + Math.sin(finalRad) * radius);
    ctx.stroke();

    // 4. Draw Object
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

// --- Note Definitions (สำหรับ Buzzer) ---
// ต้องมั่นใจว่ามีไฟล์ pitches.h หรือประกาศค่าตรงนี้
// ถ้าไม่มีไฟล์ pitches.h ให้ Uncomment บรรทัดล่างนี้แทนครับ
/*
#define NOTE_C4  262
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_D6  1175
*/
int melody[] = { NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_C6, NOTE_B5, NOTE_C6, NOTE_D6 };
int noteDurations[] = { 1, 1, 1, 1, 2, 4, 4, 2 };

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
  // สำคัญ: กำหนด Pulse Width สำหรับ Servo 270 องศา (ปกติคือ 500-2500)
  // หากใช้ Servo 180 ทั่วไป ให้ใช้ attach(SERVO_PIN) เฉยๆ ก็ได้
  myServo.attach(SERVO_PIN, 500, 2500); 
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  int sweepDir = 1;

  while (true) {
    if (isJoyMode) {
      // --- JOY MODE ---
      myServo.write(joyTargetAngle);
      currentAngle = joyTargetAngle; 
      currentDistance = measureDistance();
      vTaskDelay(10 / portTICK_PERIOD_MS); // ให้เวลา Servo ขยับนิดนึง
    } 
    else {
      // --- AUTO MODE (0-270) ---
      myServo.write(currentAngle);
      currentDistance = measureDistance();
      
      currentAngle += (sweepDir * 2); 
      
      // Limit ที่ 270
      if (currentAngle >= 270) { 
          currentAngle = 270; 
          sweepDir = -1; 
      } 
      if (currentAngle <= 0) { 
          currentAngle = 0; 
          sweepDir = 1; 
      }
      
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }
  }
}

// --- Task 2: Buzzer ---
void buzzerTask(void *parameter) {
  pinMode(BUZZER_PIN, OUTPUT);
  int melodySize = sizeof(melody) / sizeof(int);
  while (true) {
    if (currentDistance > 0 && currentDistance < 40 && isSystemArmed) {
      for (int i = 0; i < melodySize; i++) {
        // เช็คเงื่อนไขซ้ำเผื่อระยะเปลี่ยนเร็ว
        if (currentDistance >= 40 || !isSystemArmed) { noTone(BUZZER_PIN); break; }
        
        int duration = 1000 / noteDurations[i];
        tone(BUZZER_PIN, melody[i], duration);
        vTaskDelay((duration * 1.30) / portTICK_PERIOD_MS);
        noTone(BUZZER_PIN);
      }
    } else {
      vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
  }
}

// --- MQTT Callback ---
void messageReceived(String &topic, String &payload) {
  if (payload.startsWith("J")) {
    isJoyMode = true;
    int angle = payload.substring(1).toInt();
    
    // Safety Clamp: ล็อคค่าให้ไม่เกิน 270 แน่นอน
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

  xTaskCreate(radarTask, "Radar", 2048, NULL, 1, NULL);
  xTaskCreate(buzzerTask, "Buzzer", 2048, NULL, 1, NULL);
  xTaskCreate(mqttTask, "MQTT", 4096, NULL, 1, NULL);
  xTaskCreate(serverTask, "Web", 4096, NULL, 1, NULL);
}

void loop() {}