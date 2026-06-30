# DroneProject

Custom Arduino-based quadcopter controller system with a handheld Bluetooth controller, onboard stabilization algorithms, and KiCad PCB designs for both the drone electronics and the controller.

This project was developed as part of a summer program. The goal was to design and implement the main electronics and software required for a quadcopter prototype, including wireless control, sensor-based stabilization, motor mixing, safety behavior, and PCB design.

> **Project status:** Prototype / ground-tested.  
> The system was validated on the ground, but a complete stable flight test was not achieved within the project deadline. The final prototype was limited mainly by added weight, limited motor/frame lifting capacity, and the fact that the designed PCBs were not soldered and integrated before the final presentation.

## Demo

A demonstration video is available here:

[Drone Project: Custom Quadcopter Controller System](https://youtu.be/c0thseL-vnU)

## Overview

The system is divided into two main units:

1. **Drone unit**
   - Uses the frame and motors of an MJX Bugs 3 Pro drone.
   - Replaces the original electronics with a custom Arduino-based flight controller.
   - Reads IMU and compass data.
   - Applies Kalman filtering and PID control to stabilize roll, pitch, and yaw.
   - Drives four ESC-controlled brushless motors.
   - Includes safety behavior for lost communication and landing.

2. **Handheld controller unit**
   - Built around an Arduino Nano.
   - Reads joysticks, buttons, potentiometers, switches, and an MPU-6050 sensor.
   - Sends controller input packets to the drone over Bluetooth.
   - Supports normal joystick control and experimental gyro-based control modes.

```text
Handheld Controller                         Drone Unit
-------------------                         ----------
Joysticks / buttons / pots / IMU  --->  Bluetooth  --->  Arduino Nano
                                                        |
                                                        +--> IMU + compass
                                                        +--> Kalman filter
                                                        +--> PID controller
                                                        +--> Motor mixing
                                                        +--> ESCs / motors
```

## Main Features

- Arduino Nano based controller and drone firmware.
- Bluetooth communication between controller and drone.
- 18-byte controller data packet with checksum validation.
- Kalman-filtered roll and pitch angle estimation.
- PID stabilization for roll, pitch, and yaw.
- Magnetometer-based heading hold / north-facing behavior.
- Joystick control mode.
- Experimental gyro-control mode using the handheld controller orientation.
- Experimental potentiometer-based control mode.
- Motor mixing for a quadcopter X-style layout.
- Lost-signal safety behavior that slowly reduces throttle.
- Battery voltage indication using LEDs.
- KiCad PCB designs for the drone board and controller board.

## Repository Structure

```text
.
├── controller/
│   └── controller.ino
│       Firmware for the handheld controller.
│
├── drone_hardware_serial/
│   └── drone_hardware_serial.ino
│       Main drone firmware. This is the recommended drone-side version.
│       It uses the Arduino Nano hardware serial pins for Bluetooth communication,
│       which produced smoother motor behavior during testing.
│
├── drone/
│   └── drone.ino
│       Alternative drone firmware using SoftwareSerial for Bluetooth.
│       Useful for debugging because hardware Serial remains available for the
│       Arduino Serial Monitor. However, SoftwareSerial caused unstable motor
│       behavior during powered motor testing.
│
├── testing/
│   └── testing.ino
│       Experimental/testing code used during development.
│
├── PCB-Controller-main/
│       KiCad project files for the handheld controller PCB.
│
├── drone_pcb/
│       KiCad project files for the drone-side PCB.
│
└── Libraries/
        Shared KiCad symbols/footprints used by the PCB projects.
```

## Firmware Folders

### `controller/`

This folder contains the firmware uploaded to the Arduino Nano inside the handheld controller.

The controller reads:

- Two joysticks
- Four push buttons
- Two potentiometers
- Two switches
- MPU-6050 motion sensor

It packages the input data into a `Controller` struct and sends it to the drone through a Bluetooth module. The controller also performs joystick calibration and supports a gyro-control mode where the controller's tilt is used as a roll/pitch input.

### `drone_hardware_serial/`

This is the main drone firmware and the recommended version for actual motor testing.

It uses the Arduino Nano hardware serial interface for Bluetooth communication. This was more stable than the SoftwareSerial version when the motors were powered, because SoftwareSerial timing interfered with the control loop and produced unstable PWM behavior.

The firmware handles:

- Receiving controller packets over Bluetooth
- Validating received data using a checksum
- Reading IMU data
- Reading compass/magnetometer heading
- Kalman filtering for angle estimation
- PID control for stabilization
- Quadcopter motor mixing
- ESC PWM signal generation
- Battery LED indication
- Lost-communication landing behavior

### `drone/`

This is an older/alternative drone firmware version using SoftwareSerial for Bluetooth communication.

It is still useful for debugging because the hardware serial port remains available for the Serial Monitor. However, it is not the preferred version for powered motor testing because it caused unstable PWM behavior and stronger vibration during testing.

### `testing/`

This folder contains experimental code used while testing individual parts of the project.

## Communication Protocol

The controller sends a binary packet to the drone. The packet contains button states, joystick values, potentiometer values, gyro-mode roll/pitch values, and a checksum.

Simplified structure:

```cpp
struct Controller {
    uint8_t bools; // Buttons & switches
    int16_t roll, pitch; // MPU-6050 Readings
    int16_t JLx, JLy, JRx, JRy; // Joysticks input
    int16_t PL, PR; // Potentiometers
    uint8_t checksum; 
}; // 18-Bytes
```

The checksum is calculated using XOR over the packet bytes. On the drone side, packets are rejected if the checksum is invalid or if important values are outside expected limits.

The controller sends data when input changes and also sends periodic updates so the drone can detect that the connection is still alive.

## Control and Stabilization

The flight-control logic combines sensor reading, filtering, PID control, and motor mixing.

### Angle estimation

The drone uses accelerometer and gyroscope readings to estimate roll and pitch angles. A Kalman filter is used to combine the fast response of the gyroscope with the long-term correction of the accelerometer.

### PID control

PID controllers are used to reduce the difference between the target angle and the measured angle. The target angles come from either joystick input or gyro-mode controller input.

The PID outputs are then mixed into the motor signals to adjust the four motor speeds.

### Yaw / heading behavior

The drone firmware also reads magnetometer data to estimate heading. This allows heading hold behavior when there is no yaw input, and a mode for turning the drone toward north.

### Motor mixing

The drone uses four motors in a quadcopter layout:

```text
M1 = front-left
M2 = front-right
M3 = back-right
M4 = back-left
```

The firmware combines throttle, pitch, roll, and yaw values to generate the PWM signal for each motor.

## Hardware

### Drone unit

- MJX Bugs 3 Pro frame and motors
- Arduino Nano
- HW-290 sensor module
  - MPU-6050 accelerometer/gyroscope
  - Magnetometer
  - Barometer module available on the board, but not central to the current control code
- Bluetooth module
- Four ESCs
- Custom drone PCB design in KiCad
- Temporary breadboard circuit used during final testing

### Controller unit

- Arduino Nano
- Two joysticks
- Four push buttons
- Two potentiometers
- Two switches
- MPU-6050 sensor
- Bluetooth module
- Custom controller PCB design in KiCad
- Perfboard/breadboard prototype used during testing

## PCB Designs

The project includes KiCad PCB designs for both the drone and the controller.

- `PCB-Controller-main/` contains the controller PCB project.
- `drone_pcb/` contains the drone PCB project.
- `Libraries/` contains shared KiCad symbols and footprints required by the PCB projects.

The PCBs were designed and ordered, but they were not soldered, assembled, and fully tested before the project deadline. During the final testing stage, the electronics were still tested using a small breadboard mounted on the drone.

## Software Requirements

The code is written for the Arduino environment.

Expected tools/libraries:

- Arduino IDE or Arduino CLI
- Arduino Nano board support
- Standard Arduino libraries:
  - `Wire`
  - `Servo`
  - `SoftwareSerial`
- QMC5883L compass library or a compatible `QMC5883LCompass` library

Depending on the Arduino Nano board version, you may need to select either the normal ATmega328P bootloader or the old bootloader in the Arduino IDE.

## Upload Instructions

### 1. Upload the controller firmware

Open:

```text
controller/controller.ino
```

Upload it to the Arduino Nano used in the handheld controller.

### 2. Upload the recommended drone firmware

Open:

```text
drone_hardware_serial/drone_hardware_serial.ino
```

Upload it to the Arduino Nano used on the drone.

Because this version uses hardware serial for Bluetooth, disconnect the Bluetooth module from the Arduino Nano RX/TX pins while uploading if it interferes with programming.

### 3. Optional debugging firmware

For debugging, open:

```text
drone/drone.ino
```

This version uses SoftwareSerial for Bluetooth, leaving the hardware serial port available for the Arduino Serial Monitor. It is useful for troubleshooting, but it is not recommended for powered motor testing because SoftwareSerial caused unstable motor behavior during development.

## Safety Notes

This is an experimental drone-control project, not a ready-to-fly commercial flight controller.

Before testing:

- Remove propellers during firmware upload and initial motor tests.
- Secure the drone frame before powered tests.
- Verify motor order and direction.
- Verify ESC calibration and throttle range.
- Keep the drone level and still during sensor calibration.
- Test the lost-signal behavior before attempting flight.
- Do not attempt flight until the PID gains, wiring, power system, and mechanical setup are verified.

## Results

The project successfully implemented the main parts of a custom quadcopter control system:

- Wireless controller communication
- Controller input packaging
- Checksum-based packet validation
- IMU reading and calibration
- Kalman-filtered angle estimation
- PID-based stabilization logic
- Motor mixing and ESC signal generation
- PCB designs for both the controller and drone electronics

During ground testing, the electronics and algorithms responded correctly. However, stable flight was not achieved before the deadline. The main limitations were the added prototype weight, the limited lifting capacity of the reused frame/motors, and the fact that the final PCBs were not assembled and integrated in time.

## Known Limitations

- Full stable flight was not achieved.
- The drone was tested mainly on the ground.
- The PCBs were designed but not fully soldered and validated.
- The final test setup used a breadboard, which added weight and reduced reliability.
- The reused MJX Bugs 3 Pro frame and motors had limited lifting capacity with the added custom electronics.
- Bluetooth is not ideal for a final drone communication system.
- PID values require further tuning on a lighter and more reliable hardware setup.
- The SoftwareSerial drone firmware should be treated as a debugging variant, not the recommended flight-control version.

## Future Improvements

Possible next steps:

- Assemble and test the designed PCBs.
- Move to a lighter airframe or higher-thrust motors.
- Replace Bluetooth with a more suitable radio-control communication system.
- Improve power distribution and reduce electrical noise.
- Tune the PID controller during controlled flight tests.
- Add proper mechanical mounting for the IMU to reduce vibration.
- Add camera support.
- Improve failsafe behavior and pre-flight checks.
- Add more documentation for wiring and calibration.

## Author

Abdulaziz Salem Ali Salem

Project supervisor: Dr. Sütő József

## License

No license has been specified yet. If you plan to reuse this code or hardware design, please contact the author.
