#include <Wire.h>
#include <SoftwareSerial.h>

#define MAX_SPEED 1800 // Signal value for maximum motor speed is 1800us
#define MIN_SPEED 1000 // Signal value for minimum motor speed is 1000us

#define led 9

// buttons pins
#define x_pin 10
#define y_pin 12
#define a_pin 8
#define b_pin 11

// This struct is used to encapsulate all data into one type to send it via bluetooth
struct Controller {
  bool x, y, a, b;
  float roll, yaw, pitch;
  int JLx, JLy, JRx, JRy;
  int PL, PR;
};

/**************************** Global Variables ****************************/

bool new_input = false;

// To fix Joysticks' offset
int JLx_offset = 0, JLy_offset = 0, JRx_offset = 0, JRy_offset = 0;

// For gyroscope calibration
float gyro_x_bias = 0;
float gyro_y_bias = 0;
float gyro_z_bias = 0;

// For buttons debouncing
unsigned long lastTimeXChanged = millis(), lastTimeYChanged = millis(), lastTimeAChanged = millis(), lastTimeBChanged = millis();
unsigned long debounceDuration = 100;
bool lastXState, lastYState, lastAState, lastBState;


SoftwareSerial BT(3, 2);  // Bluetooth module RX -> D2, TX -> D3

// For timing bluetooth transmission
unsigned long bt_last_sent = millis(); 
unsigned long bt_sending_interval = 50; // How often to send data to drone.

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
  calibrate_gyro();
  calculate_joysticks_offset();
  
  /*
  * The calibration above takes time, to ensure that the user doesn't change the state of the controller while calibrating,
  * the red LED will blink twice when the calibration is done and the controller is ready to use.
  */

  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  delay(300);
  digitalWrite(led, LOW);
  delay(300);
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
  read_MPU();
  read_joysticks();
  read_buttons();

  // Only send if there is change, if no change after 3 seconds
  // send again to le the drone  know that the connection is working
  if (new_input || millis() - bt_last_sent >= 3000) {
  // Send data to drone via bluetooth in each time interval
    if (millis() - bt_last_sent >= bt_sending_interval) {
      // Send data to drone:
      BT.write((char *) &data, sizeof(Controller));
      bt_last_sent = millis();
    }
    new_input = false;
  }
}




/**************************** Functions ****************************/

// Reads data from the inertia unit.
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


// This function calculates how off the joysticks reading are from the middle position (512)
void calculate_joysticks_offset() {
  int samples = 50;
  for (int i = 0; i < samples; i++) {
    JRx_offset += analogRead(A3);
    JRy_offset += analogRead(A2);
    JLx_offset += analogRead(A0);
  }
  JRx_offset = JRx_offset / samples;
  JRy_offset = JRy_offset / samples;
  JLx_offset = JLx_offset / samples;
}

void read_joysticks() {

  // I decided to calculate the values to be used in the mixing formula for the motors speed here in the controller instead of doing it in the drone
  // The values of the Pitch, Roll, and Yaw have to be clipped (so that it does not change the speed too much) and centered at zero.
  // Throttle is 0 when the left joystick is pushed down. 
  int JRx_new = map(constrain(analogRead(A3) - JRx_offset, -512, 511), -512, 511, -250, 250);
  int JRy_new = map(constrain(analogRead(A2) - JRy_offset, -512, 511), -512, 511, -250, 250);
  int JLx_new = map(constrain(analogRead(A0) - JLx_offset, -512, 511), -512, 511, -250, 250);
  int JLy_new = map(analogRead(A1), 0, 1023, 1000, MAX_SPEED); // Throttle input

  if (
    abs(JRx_new - data.JRx) >= 50 ||
    abs(JRy_new - data.JRy) >= 50 ||
    abs(JLx_new - data.JLx) >= 50 ||
    abs(JLy_new - data.JLy) >= 50 
  ) {
    data.JRx = JRx_new;
    data.JRy = JRy_new;
    data.JLx = JLx_new;
    data.JLy = JLy_new;
    new_input = true;
  }

  Serial.print("Rx: " ); Serial.print(data.JRx); Serial.print("  Ry: "); Serial.print(data.JRy);
  Serial.print("    Lx: " ); Serial.print(data.JLx); Serial.print("  Ly: "); Serial.print(data.JLy);
  Serial.println();
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


// This function reads the input from the two potentiometers.
void read_potens() {
  data.PL = analogRead(A6);
  data.PR = analogRead(A7);
  // Serial.print("PL: "); Serial.print(data.PL);
  // Serial.print("    PR: "); Serial.println(data.PR);
}
