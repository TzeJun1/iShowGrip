#include <ESP32Servo.h>

Servo servos[3];
int servoPins[] = {18, 19, 21};

void setup() {
  Serial.begin(115200);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  for(int i = 0; i < 3; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2400);
    servos[i].write(0); // Start at 0
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Read the string until newline
    String data = Serial.readStringUntil('\n');
    
    // Parse the data (Format: "count,angle")
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      int count = data.substring(0, commaIndex).toInt();
      int angle = data.substring(commaIndex + 1).toInt();

      // Constrain inputs
      count = constrain(count, 0, 3);
      angle = constrain(angle, 0, 90);

      // Move servos
      for (int i = 0; i < 3; i++) {
        if (i < count) {
          servos[i].write(angle);
        } else {
          servos[i].write(0); // Reset unused servos
        }
      }
    }
  }
}