#include <Wire.h>
#include <SoftwareSerial.h>

#define MAX_SPEED 1800 // Signal value for maximum motor speed is 1800us
#define MIN_SPEED 1000 // Signal value for minimum motor speed is 1000us
#define MAX_ANGLE_INPUT 30
#define MIN_ANGLE_INPUT -30

#define led 9

// Bools masks.
#define BTN_X 0x01 // 0000 0001
#define BTN_Y 0x02 // 0000 0010
#define BTN_A 0x04 // 0000 0100
#define BTN_B 0x08 // 0000 1000

// buttons pins
#define x_pin 10
#define y_pin 12
#define a_pin 8
#define b_pin 11

// This struct collects the controller data to send it to the drone
struct Controller {
  uint8_t bools;
  int16_t roll, pitch;
  int16_t JLx, JLy, JRx, JRy;
  int16_t PL, PR;
  uint8_t checksum;
};

/**************************** Global Variables ****************************/

bool new_input = false;

// To fix Joysticks' offset
int JLx_offset = 0, JLy_offset = 0, JRx_offset = 0, JRy_offset = 0;

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


// For buttons debouncing
unsigned long lastTimeXChanged = millis(), lastTimeYChanged = millis(), lastTimeAChanged = millis(), lastTimeBChanged = millis();
unsigned long debounceDuration = 100;
bool lastXState, lastYState, lastAState, lastBState;


SoftwareSerial BT(3, 2);  // Bluetooth module TX -> 3 RX -> 2

// For timing bluetooth transmission
unsigned long bt_last_sent = millis(); 
unsigned long bt_sending_interval = 50; // How often to send data to drone. in millis

// For gyro flying mode
unsigned long gymode_time = millis();
bool switched_gymode = false;
bool led_status = LOW;

// To package controller data
Controller data;

/***************************************************************************/


void setup() {
  /*
  * When the board is powered, the red LED will be on, meaning that the controller must not be touched because it is doing calibrations.
  * After those processes are done, the red LED will blink twice, then will stay on. After that, the throttle 
  * joystick (Left joystick) should be moved down to the zero and held at that position, after some seconds the LED
  * will turn on meaning that the controller starts working and will control the drone immediately. This is a safety measure.
  */

  digitalWrite(led, HIGH);

  data.JLx = 0;
  data.JLy = MIN_SPEED;
  data.JRx = 0;
  data.JRy = 0;;

  pinMode(led, OUTPUT);
  pinMode(x_pin, INPUT_PULLUP);
  pinMode(y_pin, INPUT_PULLUP);
  pinMode(a_pin, INPUT_PULLUP);
  pinMode(b_pin, INPUT_PULLUP);

  Serial.begin(9600);
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


  // For buttons debouncing
  lastXState = digitalRead(x_pin);
  lastYState = digitalRead(y_pin);
  lastAState = digitalRead(a_pin);
  lastBState = digitalRead(b_pin);

  BT.begin(9600); // Begin bluetooth transmission.

  // Calibration processes
  delay(500);
  calculate_joysticks_offset();
  
  /*
  * The calibration above takes time, to ensure that the user doesn't change the state of the controller while calibrating,
  * the red LED will blink twice when the calibration is done and the controller is ready to use.
  */

  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);

  // The Throttle Joystick must be at lowest position in order to start the code
  while (analogRead(A1) != 0) {
    delay(3000);
  }
  digitalWrite(led, LOW);
}


void loop() {
  
  // Get data from all input devices
  read_potens();
  read_joysticks();
  read_buttons();

  // If gyro flying mode is enabled, calculate roll and pitch angles
  if ((data.bools & BTN_A) ? 1 : 0) {
    if (switched_gymode == false) {
      switched_gymode = true;
      calibrate_gyro();
      calibrate_acc();
      gymode_time = millis();
      led_status = LOW;
    }

    // Blinks the led
    if (millis() - gymode_time >= 500) {
      gymode_time = millis();
      digitalWrite(led, !led_status);
      led_status = !led_status;
    }

    // For calculating pitch and roll angles
    float dt = (micros() - last_time_kf) / 1000000.0;
    last_time_kf = micros();
    
    read_gyro();
    read_acc();

    angle_roll = constrain(kalman_filter(acc_roll, gy_roll, dt, angle_roll, bias_roll, P_roll), MIN_ANGLE_INPUT, MAX_ANGLE_INPUT);
    angle_pitch = constrain(kalman_filter(acc_pitch, gy_pitch, dt, angle_pitch, bias_pitch, P_pitch), MIN_ANGLE_INPUT, MAX_ANGLE_INPUT);

    // Ignor joystick pitch and roll inputs
    data.JRx = 0;
    data.JRy = 0;

    data.roll = (int16_t) (angle_roll * 100);
    data.pitch = (int16_t) (angle_pitch * 100);

    new_input = true;

    // Serial.print("JLy: "); Serial.print(data.JLy);
    // Serial.print("    Roll: "); Serial.print(data.roll / 100.0);
    // Serial.print("    Pitch: "); Serial.println(data.pitch / 100.0);
  } else {
    switched_gymode = false;
    digitalWrite(led, LOW);
    led_status = LOW;
    data.roll = 0;
    data.pitch = 0;
  }

  // Serial.print("Rx: " ); Serial.print(data.JRx); Serial.print("  Ry: "); Serial.print(data.JRy);
  // Serial.print("    Lx: " ); Serial.print(data.JLx); Serial.print("  Ly: "); Serial.println(data.JLy);


  // Only send if there is change, if no change after sometime
  // send again to let the drone know that the connection is working
  if (new_input || millis() - bt_last_sent >= 1000) {
  // Send data to drone via bluetooth in each time interval
    if (millis() - bt_last_sent >= bt_sending_interval) {
      // Send data to drone:
      data.checksum = compute_checksum(data);
      BT.write((char *) &data, sizeof(Controller));
      bt_last_sent = millis();
      new_input = false;
      Serial.println("Sent");
    }
  }

  // If button B is pressed, the drone will land, reset communication. The throttle must be pulled to its lowest.
  if ((data.bools & BTN_B) ? 1 : 0) {
    data.JLx = 0; data.JLy = 0; data.JRx = 0; data.JRy = 0;
    digitalWrite(led, HIGH);

    // The Throttle Joystick must be at lowest position in order to start the code
    while (analogRead(A1) != 0) {
      delay(3000);
    }
    digitalWrite(led, LOW);
  }
}




/**************************** Functions ****************************/


// For data validation in the drone side
uint8_t compute_checksum(const Controller &data) {
  const uint8_t *ptr = (const uint8_t *) &data;
  uint8_t sum = 0;

  for (size_t i = 0; i < sizeof(Controller) - 1; ++i) 
    sum ^= ptr[i];

  return sum;
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
  // TODO if the chip is placed properly on the PCB, gy_roll must be the first variable, gy_pitch must be the second one
  gy_roll = (float) gyro_x / 65.5; // Notice that we're not subracting the bias, because the Kalman filter should handle it
  gy_pitch = (float) gyro_y / 65.5;
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
  // TODO change roll and pitch so that acc_roll is the first variable, then acc_pitch, this is because the HW-290 unit is not placed in the right direction on the breadboard.
  // Signs depend on the way the chip is mounted
  acc_roll = -atan2(acc_y_clean, sqrt(acc_x_clean * acc_x_clean + acc_z_clean * acc_z_clean)) * 180 / PI;
  acc_pitch = -atan2(acc_x_clean, sqrt(acc_y_clean * acc_y_clean + acc_z_clean * acc_z_clean)) * 180 / PI;

  // Serial.print("Roll angle: "); Serial.print(roll_angle);
  // Serial.print("    Pitch angle: "); Serial.println(pitch_angle);

}


void calibrate_gyro() {
  long x_sum = 0, y_sum = 0, z_sum = 0;
  int samples = 500;

  for (int i = 0; i < samples; i++) {
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
  gyro_x_bias = (float) x_sum / samples;
  gyro_y_bias = (float) y_sum / samples;
  gyro_z_bias = (float) z_sum / samples;
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


// This function calculates how off the joysticks reading are from the middle position (512)
void calculate_joysticks_offset() {
  int samples = 100;
  unsigned long JRx_sum = 0, JRy_sum = 0, JLx_sum = 0;
  for (int i = 0; i < samples; i++) {
    JRx_sum += analogRead(A3);
    JRy_sum += analogRead(A2);
    JLx_sum += analogRead(A0);
    delay(5);
  }
  JRx_offset = JRx_sum / samples;
  JRy_offset = JRy_sum / samples;
  JLx_offset = JLx_sum / samples;
}

void read_joysticks() {

  // I decided to calculate the values to be used in the mixing formula for the motors speed here in the controller instead of doing it in the drone
  // The values of the Pitch, Roll, and Yaw have to be clipped (so that it does not change the speed too much) and centered at zero.
  // Throttle is 0 when the left joystick is pushed down. 

  int JRx_new = map(constrain(analogRead(A3) - JRx_offset, -512, 511), -512, 511, -200, 200);
  int JRy_new = -map(constrain(analogRead(A2) - JRy_offset, -512, 511), -512, 511, -200, 200);
  int JLx_new = map(constrain(analogRead(A0) - JLx_offset, -512, 511), -512, 511, -200, 200);
  int JLy_new = map(analogRead(A1), 0, 1023, MIN_SPEED, MAX_SPEED); // Throttle input

  int change_tolerance = 5; // if a value changes by this amount, update data and send to drone

  if (
    ((data.bools & BTN_A) ? 1 : 0) ||
    abs(JRx_new - data.JRx) >= change_tolerance ||
    abs(JRy_new - data.JRy) >= change_tolerance ||
    abs(JLx_new - data.JLx) >= change_tolerance ||
    abs(JLy_new - data.JLy) >= change_tolerance 
  ) {
    data.JRx = JRx_new;
    data.JRy = JRy_new;
    data.JLx = JLx_new;
    data.JLy = JLy_new;
    new_input = true;

    // Serial.print("Rx: "); Serial.print(data.JRx);
    // Serial.print("  Ry: "); Serial.println(data.JRy);
  }

  // Serial.print("Rx: " ); Serial.print(data.JRx); Serial.print("  Ry: "); Serial.print(data.JRy);
  // Serial.print("    Lx: " ); Serial.print(data.JLx); Serial.print("  Ly: "); Serial.print(data.JLy);
  // Serial.println();
}

void read_buttons() {
  data.bools = 0;
  if (!debounce(x_pin, &lastXState, &lastTimeXChanged)) data.bools |= BTN_X;
  if (!debounce(y_pin, &lastYState, &lastTimeYChanged)) data.bools |= BTN_Y;
  if (!debounce(a_pin, &lastAState, &lastTimeAChanged)) data.bools |= BTN_A;
  if (!debounce(b_pin, &lastBState, &lastTimeBChanged)) data.bools |= BTN_B;

  // Serial.print("x: "); Serial.print((data.bools & BTN_X) ? 1 : 0);
  // Serial.print("    y: "); Serial.print((data.bools & BTN_Y) ? 1 : 0);
  // Serial.print("    a: "); Serial.print((data.bools & BTN_A) ? 1 : 0);
  // Serial.print("    b: "); Serial.print((data.bools & BTN_B) ? 1 : 0);
  // Serial.print("    bools: "); Serial.println(data.bools);
}

bool debounce(int btn, bool *lastBtnState,unsigned long *lastTimeBtnChanged) {
  bool btnState = *lastBtnState;
  if (millis() - *lastTimeBtnChanged >= debounceDuration) {
    btnState = digitalRead(btn);
    if (btnState != *lastBtnState) {
      *lastBtnState = btnState;
      *lastTimeBtnChanged = millis();
      new_input = true;
    }
  }

  return btnState;
}


// This function reads the input from the two potentiometers.
void read_potens() {
  data.PL = analogRead(A6);
  data.PR = analogRead(A7);
  // Serial.print("PL: "); Serial.print(data.PL);
  // Serial.print("    PR: "); Serial.println(data.PR);
}
