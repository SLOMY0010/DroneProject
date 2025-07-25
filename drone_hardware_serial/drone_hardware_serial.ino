#include <Servo.h>
#include <SoftwareSerial.h>

#define MAX_SPEED 1800
#define MIN_SPEED 1000

// This struct is used to reconstruct the data sent by the controller
struct Controller {
  bool x, y, a, b;
  float roll, yaw, pitch;
  int JLx, JLy, JRx, JRy;
  int PL, PR;
  uint8_t checksum;
};

/************** Global Variables **************/
int throttle = 0; 
unsigned long data_waiting_time = 10000; // The drone will wait for 10s to receive from the controller, if it does not receive, it will land
unsigned long last_received = millis();

Servo M1, M2, M3, M4; // Motors configuration: M1 front-left, M2 front-right, M3 back-right, M4 back-left.
//SoftwareSerial BT(2, 7); // Bluetooth module TX -> 2 RX -> 7

Controller data;
/***********************************************/


void setup() {
  M1.attach(5); 
  M2.attach(3); 
  M3.attach(10);
  M4.attach(9); 

  M1.writeMicroseconds(2000); // Arm ESC with minimum throttle
  M2.writeMicroseconds(2000); // Arm ESC with minimum throttle
  M3.writeMicroseconds(2000); // Arm ESC with minimum throttle
  M4.writeMicroseconds(2000); // Arm ESC with minimum throttle
  delay(2000); // Give ESC time to arm

  M1.writeMicroseconds(1000); // Arm ESC with minimum throttle
  M2.writeMicroseconds(1000); // Arm ESC with minimum throttle
  M3.writeMicroseconds(1000); // Arm ESC with minimum throttle
  M4.writeMicroseconds(1000); // Arm ESC with minimum throttle
  delay(2000); // Give ESC time to arm

  //BT.begin(9600);
  Serial.begin(9600);
}


void loop() {

  // Wait for data from controller to set the motors' throttle
  if (Serial.available() >= sizeof(Controller)) {
    //Serial.println(millis() - last_received);
    // Read Bytes into the data Controller struct
    Serial.readBytes((char *) &data, sizeof(data));

    // If the checksum is correct, update throttle
    if (compute_checksum(data) == data.checksum) {
      calculate_update_throttle();
  }
    
    last_received = millis();
  }

  // If 10s went by without receiving data, and the throttle have already been changed by previously received data, land the drone.
  if (millis() - last_received >= data_waiting_time && throttle >= MIN_SPEED && throttle <= MAX_SPEED) {
    descend();
  }

  // Serial.print("Lx: "); Serial.print(data.JLx);
  // Serial.print("    Ly: "); Serial.print(data.JLy);
  // Serial.print("    Rx: "); Serial.print(data.JRx);
  // Serial.print("    Ry: "); Serial.println(data.JRy);
}




/**************************** Functions ****************************/ 


// For validating data received from the controller
uint8_t compute_checksum(const Controller &data) {
  const uint8_t *ptr = (const uint8_t *) &data;
  uint8_t sum = 0;

  for (size_t i = 0; i < sizeof(Controller) - 1; ++i) 
    sum ^= ptr[i];

  return sum;
}


void calculate_update_throttle() {
  int m1, m2, m3, m4;

  int pitch = data.JRy, roll = data.JRx, yaw = data.JLx;
  throttle = data.JLy; // A global variable

  m1 = constrain(throttle + pitch + roll + yaw, MIN_SPEED, MAX_SPEED);   // Front-left
  m2 = constrain(throttle + pitch - roll - yaw, MIN_SPEED, MAX_SPEED);   // Front-right
  m3 = constrain(throttle - pitch - roll + yaw, MIN_SPEED, MAX_SPEED);   // Back-right
  m4 = constrain(throttle - pitch + roll - yaw, MIN_SPEED, MAX_SPEED);   // Back-left

  // Update the speed of each motor
  // Serial.print("  m1: "); Serial.print(m1);
  // Serial.print("  m2: "); Serial.print(m2);
  // Serial.print("  m3: "); Serial.print(m3);
  // Serial.print("  m4: "); Serial.println(m4);
  
  update_throttle(m1, m2, m3, m4);
}


void descend() {
  // Slowly decrease the throttle of all motors equally
  for (int i = throttle; i >= MIN_SPEED; i -= 20) {
    update_throttle(i, i, i, i);
    delay(1000);
  }

  // Reset throttle
  throttle = MIN_SPEED;
}


void update_throttle(int m1, int m2, int m3, int m4) {
  M1.writeMicroseconds(m1);
  M2.writeMicroseconds(m2);
  M3.writeMicroseconds(m3);
  M4.writeMicroseconds(m4);
}
