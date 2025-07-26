#include <Wire.h>

// For gyroscope calibration
float gyro_x_bias = 0;
float gyro_y_bias = 0;
float gyro_z_bias = 0;
// For accelerometer calibration
float acc_x_bias = 0;
float acc_y_bias = 0;
float acc_z_bias = 0;

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
  // put your main code here, to run repeatedly:
  read_MPU();
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
  float acc_roll, acc_yaw, acc_pitch; // Stores acceleration in each axis

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
  float pitch_angle = atan(acc_pitch / sqrt(acc_roll * acc_roll + acc_yaw * acc_yaw)) * 180 / PI;
  float roll_angle = -atan(acc_roll / sqrt(acc_pitch * acc_pitch + acc_yaw * acc_yaw)) * 180 / PI;

  Serial.print("Roll angle: "); Serial.print(roll_angle);
  Serial.print("    Pitch angle: "); Serial.println(pitch_angle);
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