#include <Arduino.h>
#define TRIG_PIN 5
#define ECHO_PIN 18

int angle = 0;

long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  long distance = duration * 0.034 / 2;

  if (distance <= 0 || distance > 40) distance = 40;
  return distance;
}

void setup() {
  Serial.begin(9600);   // ต้องตรงกับ Processing
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long distance = measureDistance();

  Serial.print(angle);
  Serial.print(",");
  Serial.print(distance);
  Serial.print(".");

  angle++;
  if (angle >= 360) angle = 0;

  delay(30);
}
//หน้า UI ใช้processing

/*
import processing.serial.*;

Serial myPort;

int iAngle = 0;
int iDistance = 0;
String data = "";

float pixsDistance;

void setup() {
  size(800, 800);
  smooth();

  
  myPort = new Serial(this, "COM4", 9600);
  myPort.bufferUntil('.');
}

void draw() {
  background(0);
  translate(width/2, height/2);

  drawRadar();
  drawLine();
  drawObject();
  drawText();
}

void serialEvent(Serial myPort) {
  data = myPort.readStringUntil('.');
  if (data != null) {
    data = trim(data.substring(0, data.length()-1));
    int index = data.indexOf(",");
    if (index > 0) {
      iAngle = int(data.substring(0, index));
      iDistance = int(data.substring(index+1));
    }
  }
}

// ===== Radar Circle =====
void drawRadar() {
  stroke(0, 255, 0);
  noFill();

  for (int r = 100; r <= 350; r += 50) {
    ellipse(0, 0, r*2, r*2);
  }

  for (int a = 0; a < 360; a += 30) {
    line(0, 0,
         350*cos(radians(a)),
         350*sin(radians(a)));
  }
}

// ===== Sweep Line =====
void drawLine() {
  stroke(0, 255, 0);
  strokeWeight(3);

  float angleRad = radians(iAngle - 90);
  line(0, 0,
       350*cos(angleRad),
       350*sin(angleRad));
}

// ===== Object =====
void drawObject() {
  if (iDistance < 40) {
    stroke(255, 0, 0);
    strokeWeight(8);

    pixsDistance = map(iDistance, 0, 40, 0, 350);
    float angleRad = radians(iAngle - 90);

    point(pixsDistance*cos(angleRad),
          pixsDistance*sin(angleRad));
  }
}

// ===== Text =====
void drawText() {
  fill(0, 255, 0);
  textSize(16);
  textAlign(CENTER);
  text("Angle: " + iAngle + "°", 0, height/2 - 20);
  text("Distance: " + iDistance + " cm", 0, height/2);
}
*/