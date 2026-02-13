#include <Arduino.h>
#include <WiFi.h>
#include <MQTTClient.h>
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

// ---------------- MQTT ----------------
const char MQTT_BROKER_ADRRESS[] = "broker.hivemq.com";
const char MQTT_CLIENT_ID[] = "esp32-radar-87342";
const char MQTT_TOPIC[] = "esp32/radar_87342/control";

WiFiClient network;
MQTTClient mqtt(1024);
WebServer server(80);
Servo myServo;

HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

// ---------------- GLOBAL ----------------
volatile int currentAngle = 90;
volatile int currentDistance = 0;
volatile bool isSystemArmed = true;
volatile bool isJoyMode = false;
volatile int joyTargetAngle = 90;

unsigned long lastReconnectAttempt = 0;
bool isPlaying = false;

// ---------------- DISTANCE ----------------
long measureDistanceFast() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 1000);
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

  while (true) {

    if (isJoyMode) {
      myServo.write(joyTargetAngle);
      currentAngle = joyTargetAngle;
    } else {
      myServo.write(currentAngle);
      currentAngle += sweepDir * 2;
      if (currentAngle >= 180) { currentAngle = 180; sweepDir = -1; }
      if (currentAngle <= 0)   { currentAngle = 0;   sweepDir = 1; }
    }

    currentDistance = measureDistanceFast();
    vTaskDelay(20 / portTICK_PERIOD_MS);
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

// ---------------- MQTT MESSAGE ----------------
void messageReceived(String &topic, String &payload) {

  Serial.print("Received: ");
  Serial.println(payload);

  if (payload.startsWith("J")) {
    isJoyMode = true;
    int raw = payload.substring(1).toInt();
    raw = constrain(raw, 0, 180);
    joyTargetAngle = raw;
  }
  else if (payload == "AUTO") isJoyMode = false;
  else if (payload == "1") isSystemArmed = true;
  else if (payload == "0") isSystemArmed = false;
}

// ---------------- MQTT CONNECT ----------------
bool connectToMQTT() {

  if (WiFi.status() != WL_CONNECTED) return false;

  Serial.println("Connecting MQTT...");

  mqtt.setKeepAlive(20);
  mqtt.setCleanSession(true);

  if (mqtt.connect(MQTT_CLIENT_ID)) {

    Serial.println("MQTT Connected!");
    mqtt.subscribe(MQTT_TOPIC);
    Serial.println("Subscribed!");
    return true;
  }

  Serial.println("MQTT Failed");
  return false;
}

// ---------------- MQTT TASK ----------------
void mqttTask(void *parameter) {

  mqtt.begin(MQTT_BROKER_ADRRESS, 1883, network);
  mqtt.onMessage(messageReceived);

  while (true) {

    if (!mqtt.connected()) {

      unsigned long now = millis();

      if (now - lastReconnectAttempt > 3000) {
        lastReconnectAttempt = now;
        connectToMQTT();
      }
    }
    else {
      mqtt.loop();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Radar Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { background-color: #000; color: #0f0; font-family: 'Courier New', Courier, monospace; text-align: center; margin: 0; padding: 10px; display: flex; flex-direction: column; align-items: center; min-height: 100vh; }
    h1 { font-size: clamp(1.5rem, 5vw, 2rem); margin-bottom: 20px; text-shadow: 0 0 10px #0f0; }
    .radar-container { position: relative; width: 100%; max-width: 400px; display: flex; justify-content: center; }
    canvas { background-color: #111; width: 100%; height: auto; border-radius: 10px; box-shadow: 0 0 20px #0f0; border: 1px solid #333; }
    #info { font-size: 1.2rem; margin-top: 20px; font-weight: bold; padding: 10px; border: 1px solid #0f0; border-radius: 5px; width: 100%; max-width: 380px; box-sizing: border-box; background: rgba(0, 50, 0, 0.3); }
  </style>
</head>
<body>
  <h1>RADAR MONITOR (180&deg;)</h1>
  <div class="radar-container"><canvas id="radar" width="400" height="250"></canvas></div>
  <div id="info">Waiting...</div>
<script>
  var canvas = document.getElementById("radar");
  var ctx = canvas.getContext("2d");
  var width = canvas.width; var height = canvas.height;
  var cx = width/2; var cy = height - 20; var radius = (width/2)-10;

  function drawRadar(angle, dist) {
    ctx.fillStyle = "rgba(0,0,0,0.1)"; ctx.fillRect(0,0,width,height);
    
    // Grid
    ctx.strokeStyle = "#004400"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(cx, cy, radius, Math.PI, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.66, Math.PI, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, radius*0.33, Math.PI, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(10, cy); ctx.lineTo(width-10, cy); ctx.stroke();
    
    // Mirror Logic
    var rad = Math.PI + ((180 - angle) * Math.PI / 180);

    ctx.strokeStyle = "#00FF00"; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(rad) * radius, cy + Math.sin(rad) * radius);
    ctx.stroke();

    if(dist > 0 && dist < 40) {
      var pixDist = dist * (radius / 40.0);
      var ox = cx + Math.cos(rad) * pixDist;
      var oy = cy + Math.sin(rad) * pixDist;
      ctx.shadowBlur = 10; ctx.shadowColor = "red";
      ctx.fillStyle = "red"; ctx.beginPath(); ctx.arc(ox, oy, 10, 0, 2*Math.PI); ctx.fill();
      ctx.shadowBlur = 0; 
    }
  }

  setInterval(function() {
    fetch("/data").then(r => r.json()).then(d => {
      var modeText = d.mode ? "<span style='color:yellow'>JOYSTICK</span>" : "<span style='color:cyan'>AUTO</span>";
      document.getElementById("info").innerHTML = "MODE: " + modeText + "<br>ANGLE: " + d.angle + "&deg; | DIST: " + d.dist + "cm";
      drawRadar(d.angle, d.dist);
    });
  }, 40); 
</script>
</body>
</html>
)rawliteral";

// ---------------- WEB TASK ----------------
void serverTask(void *parameter) {

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

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
  Serial.begin(921600);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi Connected!");
  Serial.println(WiFi.localIP());

  xTaskCreate(radarTask, "Radar", 4096, NULL, 1, NULL);
  xTaskCreate(audioTask, "Audio", 4096, NULL, 1, NULL);
  xTaskCreate(mqttTask,  "MQTT",  6144, NULL, 1, NULL);
  xTaskCreate(serverTask,"Web",   4096, NULL, 1, NULL);
}

void loop() {}
