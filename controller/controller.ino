#include <Wire.h>
#include <SoftwareSerial.h>

#define led 9
#define x_pin 10
#define y_pin 12
#define a_pin 8
#define b_pin 11

void read_buttons();
void read_MPU();
void read_joysticks();
void read_potens();

// Packaging data to send to drone.
struct Controller {
  bool x, y, a, b;
  float roll, yaw, pitch;
  int JLx, JLy, JRx, JRy;
  int PL, PR;
};

// Global Variables:
  // For gyroscope calibration
float gyro_x_bias = 0;
float gyro_y_bias = 0;
float gyro_z_bias = 0;

  // For buttons debouncing
unsigned long lastTimeXChanged = millis(), lastTimeYChanged = millis(), lastTimeAChanged = millis(), lastTimeBChanged = millis();
unsigned long debounceDuration = 50;
bool lastXState, lastYState, lastAState, lastBState;

  // To package controller data
Controller data;

SoftwareSerial BT(3, 2);  // Bluetooth module RX -> D2, TX -> D3
unsigned long bt_last_sent = millis(); 
unsigned long bt_sending_interval = 20; // How often to send data to drone.


void setup() {
  // put your setup code here, to run once:
  digitalWrite(led, HIGH);

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

  calibrate_gyro();

  // For buttons debouncing
  lastXState = digitalRead(x_pin);
  lastYState = digitalRead(y_pin);
  lastAState = digitalRead(a_pin);
  lastBState = digitalRead(b_pin);

  BT.begin(9600); // Begin bluetooth transmission.
  delay(3000);
  digitalWrite(led, LOW);
}

void loop() {
  

  read_potens();
  read_MPU();
  read_joysticks();
  read_buttons();

  if (millis() - bt_last_sent >= bt_sending_interval) {
    // Send data:
    BT.write((char *) &data, sizeof(Controller));
    //BT.print(data.JLx); BT.print(data.JLy);
    Serial.println("Sent");
    bt_last_sent = millis();
  }

  delay(100);
}


void read_MPU() {
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  uint16_t temp;

  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;

  float roll, yaw, pitch;

  // Read data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  roll = (float) (gyro_x - gyro_x_bias) / 65.5;
  pitch = (float) (gyro_y - gyro_y_bias) / 65.5;
  yaw = (float) (gyro_z - gyro_z_bias) / 65.5;

  // Serial.print("Roll: "); Serial.print(roll);
  // Serial.print("    Yaw: "); Serial.print(yaw);
  // Serial.print("    Pitch: "); Serial.println(pitch);
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

void read_joysticks() {
  data.JRx = analogRead(A3);
  data.JRy = analogRead(A2);
  data.JLx = analogRead(A0);
  data.JLy = analogRead(A1);
  // Serial.print("Rx: " ); Serial.print(data.JRx); Serial.print("  Ry: "); Serial.print(data.JRy);
  // Serial.print("    Lx: " ); Serial.print(data.JLx); Serial.print("  Ly: "); Serial.print(data.JLy);
  // Serial.println();
}

void read_buttons() {
  data.x = debounce(x_pin, &lastXState, &lastTimeXChanged);
  data.y = debounce(y_pin, &lastYState, &lastTimeYChanged);
  data.a = debounce(a_pin, &lastAState, &lastTimeAChanged);
  data.b = debounce(b_pin, &lastBState, &lastTimeBChanged);

  // Serial.print("x: "); Serial.print(data.x);
  // Serial.print("    y: "); Serial.print(data.y);
  // Serial.print("    a: "); Serial.print(data.a);
  // Serial.print("    b: "); Serial.println(data.b);
}

bool debounce(int btn, bool *lastBtnState,unsigned long *lastTimeBtnChanged) {
  bool btnState = *lastBtnState;
  if (millis() - *lastTimeBtnChanged >= debounceDuration) {
    btnState = digitalRead(btn);
    if (btnState != *lastBtnState) {
      *lastBtnState = btnState;
      *lastTimeBtnChanged = millis();
    }
  }

  return btnState;
}


void read_potens() {
  data.PL = analogRead(A6);
  data.PR = analogRead(A7);
  // Serial.print("PL: "); Serial.print(data.PL);
  // Serial.print("    PR: "); Serial.println(data.PR);
}
