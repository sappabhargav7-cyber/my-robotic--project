#include <Wire.h>
#include <SoftwareSerial.h>

// LiDAR
#define LIDAR_ADDR 0x10
uint16_t dist;
const int SAFE_DISTANCE = 5;   // Obstacle detection
const int FOLLOW_START = 40;    // Start following if far
const int FOLLOW_STOP =20;     // Stop if too close

// Bluetooth
SoftwareSerial BT(10, 11);  // RX, TX

// Motor pins
#define IN1 7
#define IN2 8
#define IN3 4
#define IN4 5

// LED
#define LED_PIN 6

// Modes
bool autoMode = false;   // false = Manual, true = Auto Follow
char cmd = 'S';

void setup() {
  Wire.begin();
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  stopMotors();

  Serial.println("✅ System Ready (Manual Mode)");
  BT.println("✅ Send M (manual) or A (auto)");
}

void loop() {

  // Read BT commands
  if (BT.available()) {
    char c = BT.read();
    c = tolower(c);

    if (c == 'a') {
      autoMode = true;
      BT.println("Auto Follow Mode");
      Serial.println("Auto Follow Mode");
    }
    else if (c == 'm') {
      autoMode = false;
      BT.println("Manual Mode");
      Serial.println("Manual Mode");
    }
    else {
      cmd = c;   // store manual command
    }
  }

  // Read LiDAR
  dist = readLiDAR();

  // ---------------------- AUTO FOLLOW MODE ----------------------
  if (autoMode) {
    Serial.print("AUTO MODE - Distance: ");
    Serial.println(dist);

    if (dist < SAFE_DISTANCE) {
      stopMotors();
      digitalWrite(LED_PIN, HIGH);
      Serial.println("⚠ Obstacle detected!");
    }
    else if (dist > FOLLOW_START) {
      moveForward();
      digitalWrite(LED_PIN, LOW);
      Serial.println("Following... moving forward");
    }
    else if (dist < FOLLOW_STOP) {
      stopMotors();
      digitalWrite(LED_PIN, LOW);
      Serial.println("Close enough, stopping");
    }
  }

  // ---------------------- MANUAL MODE ----------------------
  else {
    Serial.print("MANUAL MODE - Distance: ");
    Serial.println(dist);

    // Safety check
    if (cmd == 'f' && dist < SAFE_DISTANCE) {
      stopMotors();
      digitalWrite(LED_PIN, HIGH);
      Serial.println("⚠ Obstacle! Can't go forward");
    }
    else {
      digitalWrite(LED_PIN, LOW);

      switch (cmd) {
        case 'f': moveForward(); break;
        case 'b': moveBackward(); break;
        case 'l': turnLeft();    break;
        case 'r': turnRight();   break;
        case 's': default: stopMotors(); break;
      }
    }
  }

  delay(100);
}

// ------------------ Motor Functions ---------------------
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ------------------ LiDAR Read ---------------------
uint16_t readLiDAR() {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;

  Wire.requestFrom(LIDAR_ADDR, 2);
  if (Wire.available() == 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    return (high << 8) | low;
  }

  return 0xFFFF;
}