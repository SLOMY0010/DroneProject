#include <Servo.h>

Servo M1, M2, M3, M4;

void set_throttle(int throttle);

void setup() {
  M1.attach(5); 
  M2.attach(3); 
  M3.attach(10);
  M4.attach(9); 

  M1.writeMicroseconds(1000); // Arm ESC with minimum throttle
  M2.writeMicroseconds(1000); // Arm ESC with minimum throttle
  M3.writeMicroseconds(1000); // Arm ESC with minimum throttle
  M4.writeMicroseconds(1000); // Arm ESC with minimum throttle
  delay(2000); // Give ESC time to arm
}

void loop() {
  M1.writeMicroseconds(1500);
  M2.writeMicroseconds(1500);
  M3.writeMicroseconds(1500);
  M4.writeMicroseconds(1500);
  delay(3000);
  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);
  delay(3000);
}


void set_throttle(int throttle) {
  M1.writeMicroseconds(throttle);
  M2.writeMicroseconds(throttle);
  M3.writeMicroseconds(throttle);
  M4.writeMicroseconds(throttle);
}