# ESP32-Based Drone Flight Controller Firmware

This repository contains the firmware for a quadcopter drone flight controller based on the ESP32 microcontroller, developed as a course project for "Introduction to Embedded Systems". The firmware integrates an MPU9250 IMU for attitude sensing, utilizes PID control algorithms for stabilization, and receives commands via Bluetooth from an Xbox Series X wireless controller.

**Project Status:** The control logic of the firmware (PID stabilization, remote control response) has been successfully validated on a test bench. However, due to hardware limitations (insufficient thrust-to-weight ratio caused by a heavy frame), stable flight was not achieved.

**Project Link:** https://github.com/chuanmin0317/ESP32_Copter

---

## Key Features

*   [x] **Hardware Integration:** Uses ESP32 as the core controller.
*   [x] **Attitude Sensing:** Reads accelerometer and gyroscope data from an MPU9250 IMU via I2C, and obtains fused Roll, Pitch, and Yaw angles using the library's built-in sensor fusion algorithm.
*   [x] **Attitude Stabilization:** Implements and tuned cascade PID control loops (Angle loop + Rate loop) to stabilize the drone's Roll, Pitch, and Yaw axes.
*   [x] **Remote Control Input:** Uses [asukiaaa's XboxSeriesXControllerESP32 library](https://github.com/asukiaaa/XboxSeriesXControllerESP32) to receive joystick and button data from an Xbox Series X controller via Bluetooth.
*   [x] **Motor Drive:** Configures the ESP32's LEDC (Hardware PWM) peripheral to generate signals and implements standard X-frame quadcopter mixing logic to drive brushless motors (requires ESCs).
*   [x] **Safety Features:** Includes a low-throttle disarm threshold (Motor Arming Threshold) and PID state reset.

## Technology Stack

*   **Core Controller:** ESP32 Dev Module (or other ESP32 board)
*   **Programming Language:** C++
*   **Development Framework:** Arduino Framework for ESP32
*   **Sensor:** MPU9250 (9-axis IMU)
*   **Communication Protocols:** I2C (for MPU9250), Bluetooth Classic (for Xbox Controller)
*   **Control Algorithm:** PID (Positional), Cascade PID
*   **Motor Control:** PWM (using ESP32 LEDC)
*   **Main Libraries:**
    *   `Wire.h` (Arduino I2C)
    *   `MPU9250.h` (<!-- Please fill in the name or source of the MPU9250 library you used -->)
    *   `XboxSeriesXControllerESP32_asukiaaa.hpp`
*   **Version Control:** Git

## Hardware Requirements

*   ESP32 Development Board
*   MPU9250 Module
*   Quadcopter Drone Frame
*   4 x Brushless Motors
*   4 x Electronic Speed Controllers (ESCs)
*   Lithium Polymer Battery (LiPo) and Power Distribution Module
*   Xbox Series X Wireless Controller (or other controller compatible with the asukiaaa library)
*   Jumper Wires, Soldering Tools, etc.

## Pin Configuration (`config.h`)

```c
/*
    motor1  motor2   (Front)
      \\      //
       \\    //
left  [ ESP32 ] right
       //    \\
      //      \\
    motor4  motor3   (Rear)
*/

// Motor PWM Output Pins
#define MOTOR1_PIN 13
#define MOTOR2_PIN 33
#define MOTOR3_PIN 32 // IMPORTANT: GPIO 34 cannot be used for output! Replaced with 32 (or choose another valid output pin).
#define MOTOR4_PIN 14

// I2C Pins for MPU9250
#define SDA_PIN 21
#define SCL_PIN 26
#define MPU9250_ADRESS 0x68 // Verify based on your MPU9250 module (AD0 pin)

// UART for Serial Debugging
#define SERIAL_BAUD_RATE 115200
```

## Installation & Compilation (Using PlatformIO or Arduino IDE)

**1. Set up Development Environment:**

*   **PlatformIO Recommended:**
    *   Install [Visual Studio Code](https://code.visualstudio.com/).
    *   Install the [PlatformIO IDE extension](https://platformio.org/platformio-ide) in VS Code.
    *   PlatformIO will automatically handle the toolchain and library dependencies.
*   **Alternatively, use Arduino IDE:**
    *   Install the [Arduino IDE](https://www.arduino.cc/en/software).
    *   Install ESP32 board support via the "Boards Manager".
    *   Manually install the required libraries:
        *   MPU9250 Library (<!-- Reminder: Please specify the library name you used -->)
        *   XboxSeriesXControllerESP32_asukiaaa Library (Download from GitHub or search in Library Manager)

**2. Get the Code:**

```bash
git clone [Your GitHub Repo Link]
cd [Your Project Directory Name]
```

### 3. Compile and Upload:

*   **Using PlatformIO:**
    *   Open the project folder in VS Code.
    *   Click the "Build" button (✓) in the PlatformIO toolbar to compile.
    *   Connect your ESP32 board to the computer.
    *   Click the "Upload" button (→) in the PlatformIO toolbar to flash the firmware onto the ESP32.
*   **Using Arduino IDE:**
    *   Open the main sketch file (e.g., `YourProjectName.ino` or `main.cpp` if you organized files).
    *   Select the correct ESP32 board model and serial port from the "Tools" menu.
    *   Click the "Verify/Compile" button.
    *   Click the "Upload" button.

## Usage Instructions

**Safety First! Always remove all propellers during initial testing and debugging!**

1.  **Upload Firmware:** Flash the compiled firmware onto the ESP32 following the steps above.
2.  **Connect Hardware:** Ensure all hardware components (IMU, ESCs, Motors, Battery) are connected correctly. **Double-check that `MOTOR3_PIN` uses an output-capable GPIO.**
3.  **Turn on Controller:** Power on the Xbox Series X controller. The ESP32's Bluetooth should automatically attempt to pair and connect (refer to the asukiaaa library documentation for initial pairing if needed).
4.  **Connect Serial Monitor:** Use the Serial Monitor in Arduino IDE or PlatformIO, set the baud rate to `115200`, and observe the debug output (e.g., connection status, IMU readings, PID values).
5.  **Arm Motors:** Move the throttle joystick (`joyLVert`) on the controller to its lowest position, then slightly upwards (above the implicit `1030` threshold in the code, recommended to define as `MOTOR_ARM_THRESHOLD`). The PID controllers should now be active, and the motors might start trying to stabilize based on the drone's orientation (if propellers are removed).
6.  **Control:**
    *   Left Stick Vertical (`joyLVert`): Controls Throttle.
    *   Right Stick Horizontal (`joyRHori`): Controls Roll.
    *   Right Stick Vertical (`joyRVert`): Controls Pitch.
    *   Left Stick Horizontal (`joyLHori`): Controls Yaw.
    *   (You should elaborate on the control direction based on the specific logic in `remote.cpp`'s `mapValue` and `RC_Analyse`).
7.  **Disarm Motors:** Move the throttle joystick to its lowest position (below the threshold). The motors will stop spinning.

## Code Structure Overview

*   `main.cpp`: Main program entry point, contains `setup()` and `loop()`, coordinates module calls.
*   `control.cpp`/`.h`: Handles triggering PID calculations, motor mixing logic, and PWM output.
*   `imu.cpp`/`.h`: Manages MPU9250 initialization, calibration application, and sensor data reading.
*   `remote.cpp`/`.h`: Responsible for Xbox controller initialization, data reading, and mapping.
*   `pid.cpp`/`.h`: Contains the PID controller object structure definition and core algorithm implementation (update, reset, cascade).
*   `config.h`: Centralizes hardware pin definitions, I2C address, baud rate, etc.
*   `MyCommon.h`/`.cpp` (or equivalent global definition area): Includes all headers, declares and defines global variables/objects, provides a global initialization function. (**Note: This is a major weakness of the current architecture**).

## Known Issues & Limitations

*   **Architectural Problem:** The code relies heavily on **global variables**, leading to **tight coupling** between modules, reducing readability, testability, and maintainability.
*   **Real-Time Issues:** The main loop uses **busy-waiting** for timing, which is inefficient and can introduce control jitter. IMU data update frequency might not synchronize with the main loop.
*   **Sensor Calibration:** IMU calibration values are **hard-coded**, failing to adapt to individual sensor variations or environmental changes.
*   **Parameter Management:** PID gains, limit values, RC mapping factors, etc., use **magic numbers**, and PID gains require **manual tuning** to function.
*   **Robustness:** Error handling is basic; RC **deadzone** is not handled; PID **derivative term** implementation might be sensitive to noise and suffer from derivative kick.

## Potential Future Enhancements (To-Do)

*   [ ] **Refactor Architecture:** Reduce or eliminate global variable dependency. Consider using classes (OOP) to encapsulate module functionality or pass state via function parameters/return values.
*   [ ] **Improve Loop Scheduling:** Replace busy-waiting with FreeRTOS tasks and `vTaskDelayUntil` or non-blocking delays (`yield()`) for precise and efficient timing.
*   [ ] **Synchronize Data Flow:** Ensure sensor data updates are synchronized with the control loop.
*   [ ] **Correct Hardware Pin Config:** Ensure all pin definitions are correct and suitable for their intended use.
*   [ ] **Implement Runtime Calibration:** Add an IMU calibration routine and store results in the ESP32's non-volatile memory (Flash).
*   [ ] **Parameterize Configuration:** Replace magic numbers with constants or a configuration system; implement runtime PID tuning or loading from storage.
*   [ ] **Enhance Robustness:** Add RC deadzone handling; improve PID derivative calculation (e.g., derivative on measurement, filtering); implement more robust error handling.
*   [ ] **Write Tests:** Implement unit tests for critical algorithms (PID, mixing) if feasible.
*   [ ] **Add More Features:** E.g., Altitude Hold (requires barometer), GPS Position Hold, Flight Mode switching.
