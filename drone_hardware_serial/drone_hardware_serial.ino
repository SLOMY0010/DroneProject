#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

#define MAX_SPEED 1800
#define MIN_SPEED 1000
#define MAX_ANGLE_INPUT 30
#define MIN_ANGLE_INPUT -30
#define BATTERY_PIN A7
#define BAT_R_LED 4
#define BAT_G_LED 6

// Bools masks. For all boolean inputs.
#define BTN_X 0x01 // 0000 0001
#define BTN_Y 0x02 // 0000 0010
#define BTN_A 0x04 // 0000 0100
#define BTN_B 0x08 // 0000 1000

// MPU-6050 registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define USER_CTRL 0x6A
#define INT_PIN_CFG 0x37

#define PID_SCALE_FACTOR 2.5 // This is used to scale the PID output to proper motor signal

// This struct is used to reconstruct the data sent by the controller
struct Controller {
  uint8_t bools;
  int16_t roll, pitch;
  int16_t JLx, JLy, JRx, JRy;
  int16_t PL, PR;
  uint8_t checksum;
};

/************** Global Variables **************/
int throttle = 0; 
unsigned long data_waiting_time = 3000; // The drone will wait for this amoung of milliseconds to receive from the controller, if it does not receive, it will land
unsigned long last_received = millis();
bool transmission_started = false; 

Servo M1, M2, M3, M4; // Motors configuration: M1 front-left, M2 front-right, M3 back-right, M4 back-left.
//SoftwareSerial BT(2, 7); // Bluetooth module TX -> 2 RX -> 7

// For gyroscope calibration
float gyro_x_bias = 0;
float gyro_y_bias = 0;
float gyro_z_bias = 0;

// For accelerometer calibration
float acc_x_bias = 0;
float acc_y_bias = 0;
float acc_z_bias = 0;

// For the Kalman filter

  unsigned long last_time_kf = micros();

  // For roll
  float angle_roll = 0.0;
  float bias_roll = 0.0;
  float P_roll[2][2] = { {1, 0}, {0, 1} };

  // For pitch
  float angle_pitch = 0.0;
  float bias_pitch = 0.0;
  float P_pitch[2][2] = { {1, 0}, {0, 1} };

  // Kalman tuning constants (shared)
  float Q_angle = 0.004;
  float Q_bias  = 0.003;
  float R_measure = 0.04;

  float gy_roll, gy_yaw, gy_pitch; // Stores rotation rates in each axis (angular velocity)
  float acc_roll, acc_yaw, acc_pitch; // Stores acceleration in each axis

// For PID control
  // For roll
  float target_roll = 0.0;
  float prev_error_roll = 0.0;
  float integral_roll = 0.0;

  float Kp_roll = 1.0;
  float Ki_roll = 0.05;
  float Kd_roll = 0.02;

  unsigned long last_time_pid_roll = micros();

  // For pitch
  float target_pitch = 0.0;
  float prev_error_pitch = 0.0;
  float integral_pitch = 0.0;

  float Kp_pitch = 1.0;
  float Ki_pitch = 0.05;
  float Kd_pitch = 0.02;

  unsigned long last_time_pid_pitch = micros();

  // For yaw
  float target_yaw;
  float prev_error_yaw = 0.0;
  float integral_yaw = 0.0;

  float Kp_yaw = 0.2;
  float Ki_yaw = 0.01;
  float Kd_yaw = 0;

  bool got_target_yaw = false;

  unsigned long last_time_pid_yaw = micros();

// For the Magnetometer (compass)
  QMC5883LCompass compass;
  float heading;
  float declination_angle = 6.1666;


Controller data;
/***********************************************/


void setup() {
  pinMode(BAT_G_LED, OUTPUT);
  pinMode(BAT_R_LED, OUTPUT);

  data.JLy = MIN_SPEED;
  data.JLx = 0;
  data.JRx = 0;
  data.JRy = 0;
  data.bools = 0; // No gyro flight mode

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

  Wire.setClock(400000); // For the MPU-6050
  Wire.begin();

  delay(250);

  // Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(0x68);

  // MPU Configuration
  Wire.beginTransmission(0x68); // 0x68 is the address of the MPU-6050
  Wire.write(0x1A); // Configures the digital low-pass filter
  Wire.write(0x05); // Chooses a low-pass filter with 10Hz cuttoff frequency
  Wire.endTransmission();

  // Set sensetivity
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8); // Full scale range: 500 degs/s or LSB 65.5
  Wire.endTransmission();

  enable_mpu_bypass(); // To allow connect the magnetometer to the main I2C bus

  calibrate_gyro();
  calibrate_acc();
  compass.init();

  Serial.begin(9600);
}


void loop() {

  battery_status(); // Checks battery voltage and update led color accordingly.

  // Save old data in case received data is corrupted
  Controller old_data = data;
  if (Serial.available() >= sizeof(Controller)) {
    Serial.readBytes(((char *) &data), sizeof(Controller)); // Read rest of data
    last_received = millis();
    analogWrite(BAT_R_LED, 0);
    analogWrite(BAT_G_LED, 175);

    if (compute_checksum(data) == data.checksum) {
      transmission_started = true;
    } else {
      data = old_data;
      analogWrite(BAT_R_LED, 255);
      analogWrite(BAT_G_LED, 255); // LED will blink red if data was rejected
    }
  }
    
  if (data.JLy == 0) {
    analogWrite(BAT_R_LED, 0);
    analogWrite(BAT_G_LED, 255);
  }

  // if (transmission_started) {
  //   // This function handles all movement and calculations
  //   calculate_update_throttle();
  // }
  calculate_update_throttle();
  // Serial.print("roll: "); Serial.print(angle_roll);
  // Serial.print("    pitch: "); Serial.println(angle_pitch);

  // If 10s went by without receiving data, and the throttle have already been changed by previously received data, land the drone.
  if ((millis() - last_received >= data_waiting_time && transmission_started) || ((data.bools & BTN_B) ? 1 : 0)) { // Pressing B lands the drone
    transmission_started = false;
    descend();
  }
}


/**************************** Functions ****************************/

void enable_mpu_bypass() {

  // Disable I2C master (ensure aux bus can be bypassed)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(USER_CTRL);
  Wire.write(0x00); // I2C_MST_EN=0
  Wire.endTransmission();

  // Enable BYPASS_EN so AUX_DA/AUX_CL connect to SDA/SCL
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02); // BYPASS_EN=1
  Wire.endTransmission();
  delay(10);

}

void calculate_update_throttle() {
  int m1, m2, m3, m4;

  // Pitch and Roll inputs are mapped to -30 to 30 degrees range
  int pitch_input = map(data.JRy, -200, 200, MIN_ANGLE_INPUT, MAX_ANGLE_INPUT), roll_input = map(data.JRx, -200, 200, MIN_ANGLE_INPUT, MAX_ANGLE_INPUT), yaw_input = data.JLx;
  throttle = data.JLy; // A global variable
  
  // If A is pressed, gyro mode is enabled, roll and pitch joystick inputs are ignored.
  if ((data.bools & BTN_A) ? 1 : 0) {
    target_roll = data.roll / 100.0;
    target_pitch = data.pitch / 100.0;
  } else {
    target_roll = roll_input;
    target_pitch = pitch_input;
  }

  // Get angles measurments using the Kalman Filter
  unsigned long current_time = micros();
  float dt = (current_time - last_time_kf) / 1000000.0;
  last_time_kf = current_time;

  read_gyro();
  read_acc();

  angle_roll = kalman_filter(acc_roll, gy_roll, dt, angle_roll, bias_roll, P_roll);
  angle_pitch = kalman_filter(acc_pitch, gy_pitch, dt, angle_pitch, bias_pitch, P_pitch);

  // Get PID output
  float pid_output_pitch = pid_update(target_pitch, angle_pitch, (current_time - last_time_pid_pitch) / 1000000.0, Kp_pitch, Ki_pitch, Kd_pitch, prev_error_pitch, integral_pitch);
  last_time_pid_pitch = current_time;
  float pid_output_roll = pid_update(target_roll, angle_roll, (current_time - last_time_pid_roll) / 1000000.0, Kp_roll, Ki_roll, Kd_roll, prev_error_roll, integral_roll);
  last_time_pid_roll = current_time;

  read_compass(); // Calculates and sets the heading variable

  // If X was pressed, drone must face north. Facing north will be interrupted if there is input on JLx
  if ((data.bools & BTN_X) ? 1 : 0) {
    target_yaw = 0.0; // North
    got_target_yaw = true;
  }

  // Since we are not controlling the angles of yaw directly, we will use PID only to keep the drone face the same direction, when there is no yaw input
  if (data.JLx == 0 && got_target_yaw == false) {
    target_yaw = heading;
    got_target_yaw = true;
  } else if (data.JLx != 0) {
    got_target_yaw = false;
  }

  if (got_target_yaw) {
    float pid_output_yaw = pid_update(
      (target_yaw > 180) ? target_yaw - 360 : target_yaw, 
      (heading > 180) ? heading - 360 : heading, 
      (current_time - last_time_pid_yaw) / 1000000.0, 
      Kp_yaw, Ki_yaw, Kd_yaw, 
      prev_error_yaw, 
      integral_yaw
    );

    yaw_input = (int) (constrain(pid_output_yaw, -30, 30) * PID_SCALE_FACTOR * 2);

    last_time_pid_yaw = current_time;
  }

  // Scale the pid output to proper motor signals:
  int roll = (int) (pid_output_roll * PID_SCALE_FACTOR);
  int pitch = (int) (pid_output_pitch * PID_SCALE_FACTOR);

  m1 = constrain(throttle + pitch + roll + yaw_input, MIN_SPEED, MAX_SPEED);   // Front-left
  m2 = constrain(throttle + pitch - roll - yaw_input, MIN_SPEED, MAX_SPEED);   // Front-right
  m3 = constrain(throttle - pitch - roll + yaw_input, MIN_SPEED, MAX_SPEED);   // Back-right
  m4 = constrain(throttle - pitch + roll - yaw_input, MIN_SPEED, MAX_SPEED);   // Back-left

  // Serial.print("m1: "); Serial.print(m1); Serial.print("    m2: "); Serial.print(m2);
  // Serial.print("    m3: "); Serial.print(m3); Serial.print("    m4: "); Serial.println(m4);

  // Update the speed of each motor
  update_throttle(m1, m2, m3, m4);
}


float pid_update(float target, float measured_angle, float dt, float Kp, float Ki, float Kd, float &prev_error, float &integral) {
  float error = target - measured_angle;

  if (abs(error) < 0.5) integral = 0.0;

  integral += error * dt;

  float max_integral = 100;
  if (integral > max_integral) integral = max_integral;
  if (integral < -max_integral) integral = -max_integral;
  
  float derivative = (error - prev_error) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  prev_error = error;
  return output;
}


float kalman_filter(float acc_angle, float gyro_rate, float dt, float &angle, float &bias, float P[2][2]) {

  // Prediction step
  angle += dt * (gyro_rate - bias);

  // Update error covariance
  P[0][0] += dt * (dt * P[1][1] - P[1][0] - P[0][1] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Calculate the Kalman gain
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Measurement update
  float y = acc_angle - angle;
  angle += K[0] * y;
  bias  += K[1] * y;

  // Update P
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}


void update_throttle(int m1, int m2, int m3, int m4) {
  M1.writeMicroseconds(m1);
  M2.writeMicroseconds(m2);
  M3.writeMicroseconds(m3);
  M4.writeMicroseconds(m4);
}


void read_gyro() {
  // To store raw data from the gyroscope
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  

  // Read rotation rates from the gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  // Rates:
  // TODO if the chip is placed properly on the PCB, swap gy_roll and gy_pitch
  gy_pitch = (float) gyro_x / 65.5; // Notice that we're not subracting the bias, because the Kalman filter should handle it
  gy_roll = (float) gyro_y / 65.5;
  gy_yaw = (float) gyro_z / 65.5;

  // Serial.print("Roll: "); Serial.print(gy_roll);
  // Serial.print("    Yaw: "); Serial.print(gy_yaw);
  // Serial.print("    Pitch: "); Serial.println(gy_pitch);

}


void read_acc() {
  // To store raw data from the accelerometer
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;

  // Read data from the accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  // Read bytes
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();

  float acc_x_clean = (float) acc_x / 4096 - acc_x_bias; 
  float acc_y_clean= (float) acc_y / 4096 - acc_y_bias;
  float acc_z_clean = (float) acc_z / 4096 - acc_z_bias + 1; // Because when the drone is idle, it will have gravitational acceleration along the z axis

  // This calculates the angles by using the accelerometer
  // TODO change roll and pitch variable names, this is because the HW-290 unit is not placed in the right direction on the breadboard
  // Signs depend on the way the chip is mounted
  acc_pitch = atan2(acc_y_clean, sqrt(acc_x_clean * acc_x_clean + acc_z_clean * acc_z_clean)) * 180 / PI;
  acc_roll = -atan2(acc_x_clean, sqrt(acc_y_clean * acc_y_clean + acc_z_clean * acc_z_clean)) * 180 / PI;

  // Serial.print("Roll angle: "); Serial.print(roll_angle);
  // Serial.print("    Pitch angle: "); Serial.println(pitch_angle);

}


void read_compass() {
  // Read heading from compass
  compass.read();
  int mx = compass.getX(); int my = compass.getY(); int mz = compass.getZ();

  // Calibration values: -1062,2191,-757,2407,-1631,1736
  int mx_min = -1062, mx_max = 2191, my_min = -757, my_max = 2407, mz_min = -1631, mz_max = 1736;

  // Compute offsets
  float offset_x = (mx_max + mx_min) / 2.0, offset_y = (my_max + my_min) / 2.0, offset_z = (mz_max + mz_min) / 2.0;

  // Compute scale factors
  float range_x = (mx_max - mx_min) / 2.0, range_y = (my_max - my_min) / 2.0, range_z = (mz_max - mz_min) / 2.0;
  float avg_range = (range_x + range_y + range_z) / 3.0;

  // Scale and subtract offsets
  float mx_cal = (mx - offset_x) * (avg_range / range_x);
  float my_cal = (my - offset_y) * (avg_range / range_y);
  float mz_cal = (mz - offset_z) * (avg_range / range_z);

  float roll_rad = -angle_roll * PI / 180.0; float pitch_rad = -angle_pitch * PI / 180.0;

  float mxh = mx_cal * cos(pitch_rad) + mz_cal * sin(pitch_rad);
  float myh = mx_cal * sin(roll_rad) * sin(pitch_rad) + my_cal * cos(roll_rad) - mz_cal * sin(roll_rad) * cos(pitch_rad);

  heading = atan2(myh, mxh) * 180.0 / PI + declination_angle;
  if (heading < 0) heading += 360.0;
}


void calibrate_gyro() {
  long x_sum = 0, y_sum = 0, z_sum = 0;

  for (int i = 0; i < 2000; i++) {
    // Read data
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 6);
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    x_sum += gx;
    y_sum += gy;
    z_sum += gz;
  }
  gyro_x_bias = (float) x_sum / 2000;
  gyro_y_bias = (float) y_sum / 2000;
  gyro_z_bias = (float) z_sum / 2000;
}


void calibrate_acc() {
  long x_sum = 0, y_sum = 0, z_sum = 0;
  int samples = 200;

  for (int i = 0; i < samples; i++) {
    // Read data
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    int16_t acc_x = Wire.read() << 8 | Wire.read();
    int16_t acc_y = Wire.read() << 8 | Wire.read();
    int16_t acc_z = Wire.read() << 8 | Wire.read();

    x_sum += acc_x;
    y_sum += acc_y;
    z_sum += acc_z;
  }

  acc_x_bias = ((float) x_sum / samples) / 4069.0;
  acc_y_bias = ((float) y_sum / samples) / 4069.0;
  acc_z_bias = ((float) z_sum / samples) / 4069.0;
}


void descend() {
  // Slowly decrease the throttle of all motors equally
  for (int i = throttle; i >= MIN_SPEED; i -= 20) {
    update_throttle(i, i, i, i);
    delay(1000);
  }

  // Reset throttle
  data.JLy = MIN_SPEED;
  throttle = MIN_SPEED;
}


// For validating data received from the controller
uint8_t compute_checksum(const Controller &data) {
  const uint8_t *ptr = (const uint8_t *) &data;
  uint8_t sum = 0;

  for (size_t i = 0; i < sizeof(Controller) - 1; ++i) 
    sum ^= ptr[i];

  // In case some data violates limits but still seems valid in the checksum, set checksum to 0 to reject it
  if (data.JLy < MIN_SPEED || data.JLy > MAX_SPEED ||
      data.JLx < -200 || data.JLx > 200 ||
      data.JRx < -200 || data.JRx > 200 ||
      data.JRy < -200 || data.JRy > 200 ||
      (data.roll / 100.0) < MIN_ANGLE_INPUT || (data.roll / 100.0) > MAX_ANGLE_INPUT ||
      (data.pitch / 100.0) < MIN_ANGLE_INPUT || (data.pitch / 100.0) > MAX_ANGLE_INPUT) 
    {
      return -1;  
    } 

  return sum;
}


float battery_status() {
  int pin_reading = analogRead(BATTERY_PIN);
  float pin_voltage = pin_reading * 5.0 / 1023.0;
  float battery_voltage = pin_voltage * 2; // Because the voltage devider is a 1/2 divider
  
  if (battery_voltage <= 8.4 && battery_voltage > 7.4) {
    // 0 -> fully on, 255 -> fully off
    analogWrite(BAT_R_LED, 255);
    analogWrite(BAT_G_LED, 0);
  } else if (battery_voltage <= 7.4 && battery_voltage > 7.0) {
    analogWrite(BAT_R_LED, 0);
    analogWrite(BAT_G_LED, 0);
  } else if (battery_voltage <= 7.0 && battery_voltage > 6.0) {
    analogWrite(BAT_R_LED, 0);
    analogWrite(BAT_G_LED, 175);
  } else if (battery_voltage <= 6.0) {
    analogWrite(BAT_R_LED, 0);
    analogWrite(BAT_G_LED, 255);
  }
}