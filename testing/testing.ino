#include <Wire.h>

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

  float Kp_roll = 1.5;
  float Ki_roll = 0.05;
  float Kd_roll = 0.2;

  unsigned long last_time_pid_roll = micros();

  // For pitch
  float target_pitch = 0.0;
  float prev_error_pitch = 0.0;
  float integral_pitch = 0.0;

  float Kp_pitch = 1.5;
  float Ki_pitch = 0.05;
  float Kd_pitch = 0.2;

  unsigned long last_time_pid_pitch = micros();

void setup() {
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

  calibrate_gyro();
  calibrate_acc();

  Serial.begin(9600);
}

void loop() {
  float dt = (micros() - last_time_kf) / 1000000.0;
  last_time_kf = micros();

  read_gyro();
  read_acc();

  angle_roll = kalman_filter(acc_roll, gy_roll, dt, angle_roll, bias_roll, P_roll);
  angle_pitch = kalman_filter(acc_pitch, gy_pitch, dt, angle_pitch, bias_pitch, P_pitch);

  float pid_output_pitch = pid_update(target_pitch, angle_pitch, (micros() - last_time_pid_pitch) / 1000000.0, Kp_pitch, Ki_pitch, Kd_pitch, prev_error_pitch, integral_pitch);
  last_time_pid_pitch = micros();
  float pid_output_roll = pid_update(target_roll, angle_roll, (micros() - last_time_pid_roll) / 1000000.0, Kp_roll, Ki_roll, Kd_roll, prev_error_roll, integral_roll);
  last_time_pid_roll = micros();

  Serial.print("PID Roll: "); Serial.print(pid_output_roll);
  Serial.print("    PID Pitch: "); Serial.println(pid_output_pitch);
  
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
























// Reads data from the inertia unit.
void read_MPU() {

  // To store raw data from the gyroscope
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  // To store raw data from the accelerometer
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;

  float gy_roll, gy_yaw, gy_pitch; // Stores rotation rates in each axis (angular velocity)

  // Read rotation rates from the gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  gy_roll = (float) (gyro_x - gyro_x_bias) / 65.5;
  gy_pitch = (float) (gyro_y - gyro_y_bias) / 65.5;
  gy_yaw = (float) (gyro_z - gyro_z_bias) / 65.5;

  // Serial.print("Roll: "); Serial.print(roll);
  // Serial.print("    Yaw: "); Serial.print(yaw);
  // Serial.print("    Pitch: "); Serial.println(pitch);

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

  acc_roll = (float) acc_x / 4096 - acc_x_bias;
  acc_pitch= (float) acc_y / 4096 - acc_y_bias;
  acc_yaw = (float) acc_z / 4096 - acc_z_bias + 1; // Because when the drone is idle, it will have gravitational acceleration along the z axis

  // Serial.print("acc_x: "); Serial.print(acc_roll);
  // Serial.print("    acc_y: "); Serial.print(acc_pitch);
  // Serial.print("    acc_z: ");Serial.println(acc_yaw);

  // This calculates the angles by using the accelerometer
  // TODO change roll and pitch variable names, this is because the HW-290 unit is not placed in the right direction on the breadboard
  // Signs depend on the way the chip is mounted
  float pitch_angle = -atan2(acc_pitch, sqrt(acc_roll * acc_roll + acc_yaw * acc_yaw)) * 180 / PI;
  float roll_angle = -atan2(acc_roll, sqrt(acc_pitch * acc_pitch + acc_yaw * acc_yaw)) * 180 / PI;

  Serial.print("Roll angle: "); Serial.print(roll_angle);
  Serial.print("    Pitch angle: "); Serial.println(pitch_angle);
}
