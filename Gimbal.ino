#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;

// --- Servo Setup ---
Servo servoRoll;
Servo servoPitch;
const int rollPin = 13;
const int pitchPin = 14;

// --- OFFSET ADJUSTMENT ---
// Adjust these numbers (e.g., -10, 5, 12) until the gimbal is parallel 
// to the MPU6050 when the sensor is sitting flat on a table.
float rollOffset = 0.0;  
float pitchOffset = 0.0; 

// --- Smoothing/Filter ---
float filterWeight = 0.1; 
float smoothRoll = 90;
float smoothPitch = 90;

void setup(void) {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoRoll.setPeriodHertz(50);
  servoPitch.setPeriodHertz(50);
  servoRoll.attach(rollPin, 500, 2400);
  servoPitch.attach(pitchPin, 500, 2400);

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 1. Calculate angles from MPU
  float rollAngle = atan2(a.acceleration.y, a.acceleration.z) * 57.2958;
  float pitchAngle = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958;

  // 2. Add your custom offsets for mechanical alignment
  float correctedRoll = rollAngle + rollOffset;
  float correctedPitch = pitchAngle + pitchOffset;

  // 3. Map to Servo Range
  int targetRoll = map(correctedRoll, -90, 90, 180, 0); 
  int targetPitch = map(correctedPitch, -90, 90, 0, 180);

  // 4. Apply Smoothing
  smoothRoll = (targetRoll * filterWeight) + (smoothRoll * (1.0 - filterWeight));
  smoothPitch = (targetPitch * filterWeight) + (smoothPitch * (1.0 - filterWeight));

  // 5. Write to Servos
  servoRoll.write(constrain((int)smoothRoll, 0, 180));
  servoPitch.write(constrain((int)smoothPitch, 0, 180));

  delay(10); 
}