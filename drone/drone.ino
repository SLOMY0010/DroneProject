#include <Servo.h>
#include <SoftwareSerial.h>

#define MAX_THROTTLE 1800
#define MIN_THROTTLE 1150

void set_throttle(int throttle);
void update_throttle(int m1, int m2, int m3, int m4);
void calculate_update_throttle();

Servo M1, M2, M3, M4;
SoftwareSerial BT(2, 7); // Bluetooth module RX -> 7 TX -> 2

// Packaging data received from controller
struct Controller {
  bool x, y, a, b;
  float roll, yaw, pitch;
  int JLx, JLy, JRx, JRy;
  int PL, PR;
};

Controller data;

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

  BT.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if (BT.available() >= sizeof(Controller)) {
    while (BT.available()) {

      // Read data into the data Controller struct
      BT.readBytes((char *) &data, sizeof(data));
      calculate_update_throttle();
    }
  }
  Serial.print("Lx: "); Serial.print(data.JLx);
  Serial.print("    Ly: "); Serial.print(data.JLy);
  Serial.print("    Rx: "); Serial.print(data.JRx);
  Serial.print("    Ry: "); Serial.println(data.JRy);
}


void calculate_update_throttle() {
  int m1, m2, m3, m4;

  m1 = throttle + pitch + roll + yaw   // Front-left
  m2 = throttle + pitch - roll - yaw   // Front-right
  m3 = throttle - pitch - roll + yaw   // Back-right
  m4 = throttle - pitch + roll - yaw   // Back-left
}




void update_throttle(int m1, int m2, int m3, int m4) {
  M1.writeMicroseconds(m1);
  M2.writeMicroseconds(m2);
  M3.writeMicroseconds(m3);
  M4.writeMicroseconds(m4);
}


void set_throttle(int throttle) {
  M1.writeMicroseconds(throttle);
  M2.writeMicroseconds(throttle);
  M3.writeMicroseconds(throttle);
  M4.writeMicroseconds(throttle);
}