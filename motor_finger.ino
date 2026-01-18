#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Servo leftServo;
Servo rightServo;

// --- CONFIGURATION ---
const int LEFT_PIN = 19;
const int RIGHT_PIN = 23;

// Define specific angles to avoid confusion
const int LEFT_OPEN_ANGLE = 0;
const int LEFT_CLOSE_ANGLE = 90;

const int RIGHT_OPEN_ANGLE = 90;
const int RIGHT_CLOSE_ANGLE = 0; // 180 - 90 = 90

void setup() {
  Serial.begin(9600);
  
  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
  
  // Start Position: Both OPEN
  leftServo.write(LEFT_OPEN_ANGLE);   // Goes to 0
  rightServo.write(RIGHT_OPEN_ANGLE); // Goes to 180
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // --- LEFT HAND LOGIC (0 -> 90) ---
    if (command == "LC" || command == "RC") {
      leftServo.write(LEFT_CLOSE_ANGLE); // Moves Clockwise to 90
      rightServo.write(RIGHT_CLOSE_ANGLE);
    } 
    else if (command == "LO" || command == "RO") {
      leftServo.write(LEFT_OPEN_ANGLE);  // Moves back to 0
      rightServo.write(RIGHT_OPEN_ANGLE);  // Moves back to 180
    }

  }
  
}