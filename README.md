![top](https://github.com/user-attachments/assets/590094fa-ea04-48e8-b4dc-d062c1ab11d0)

<p align="center">
  <a href="https://www.logiclaboratories.tech">
    <img src="https://img.shields.io/badge/mywebsite-1B1F24?style=for-the-badge&logo=vercel&logoColor=white" />
  </a>
  <a href="https://www.linkedin.com/in/kurtaxlsaludo/">
    <img src="https://img.shields.io/badge/linked%20in-1B1F24?style=for-the-badge&logo=linkedin&logoColor=white" />
  </a>
  <a href="mailto:axlsaludo@proton.me">
     <img src="https://img.shields.io/badge/contact%20me-1B1F24?style=for-the-badge&logo=protonmail&logoColor=white" />
  </a>
  <a href="https://www.cloudskillsboost.google/public_profiles/22d7733e-0133-4057-affc-6e6f5ab6f8e9">
    <img src="https://img.shields.io/badge/gcp%20dev-1B1F24?style=for-the-badge&logo=google-cloud&logoColor=white" />
  </a>
</p>

# Color Detection Robot

This repository contains the code and setup instructions for a **Color Detection Robot**, designed by Logic Laboratories. The robot detects a specific color (such as a tennis ball) using a camera and HSV color calibration. Once detected, it moves toward the object by controlling motors through an Arduino microcontroller, connected via serial communication.

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Software Requirements](#software-requirements)
3. [File Structure](#file-structure)
4. [Setup Instructions](#setup-instructions)
5. [Usage](#usage)
6. [Troubleshooting](#troubleshooting)
7. [License](#license)

## Hardware Requirements
- **Arduino Uno**: To control the robot motors.
- **Motor Driver (we used TB6612FNG)**: To drive the robot’s motors.
- **Raspberry Pi**: To run the color detection code.
- **Webcam**: For real-time color detection.
- **Robot Chassis**: Motorized chassis for movement.
- **Tennis ball (or other color object)**: The object the robot will track.

## Software Requirements
- **Python**: For running the main detection code.
- **OpenCV**: For image processing (`cv2` library).
- **Numpy**: For numerical processing.
- **PySerial**: For serial communication between the Raspberry Pi/Laptop and the Arduino.

### Python Libraries Installation
Install the required libraries by running:
```bash
pip install opencv-python numpy pyserial
```

## File Structure
```
|-- main.py               # Main color detection script
|-- threshold.py          # HSV color thresholding and calibration
|-- arduino_code.ino      # Arduino sketch for motor control
|-- color_thresholds.txt  # Exported HSV values (generated after calibration)
```

## Setup Instructions

### Step 1: Arduino Setup
1. **Upload the `arduino_code.ino`** to your Arduino board using the Arduino IDE.
2. Connect the Arduino to your motor driver and chassis according to the pin definitions in the code.
   - The Arduino reads movement commands from serial input (e.g., `F` for forward, `B` for backward).
   - Motors are driven using PWM values received from the main Python script.

### Step 2: Camera Color Calibration
1. **Run `threshold.py`** to calibrate the HSV color detection for the object (e.g., a tennis ball):
    ```bash
    python3 threshold.py
    ```
2. Adjust the HSV sliders in the UI until the mask correctly detects the object.
3. The final calibrated values are automatically saved to `color_thresholds.txt`.

### Step 3: Color Detection and Motor Control
1. Connect the Arduino to your Raspberry Pi or computer.
2. **Run `main.py`** to begin the detection and control process:
    ```bash
    python3 main.py
    ```
   - The robot will begin tracking the color and adjusting its position accordingly.
   - You can stop the script anytime by pressing `q`.

### Step 4: Serial Connection
Ensure that the correct serial port is used when initializing the camera detector:
```python
detector = CameraDetector('/dev/ttyACM0', 9600)
```
Adjust `/dev/ttyACM0` to match your Arduino's serial port if needed.

## Usage

### Commands
- **Forward Movement (`F + PWM`)**: Moves the robot forward with the specified PWM.
- **Backward Movement (`B + PWM`)**: Moves the robot backward.
- **Left Turn (`L + PWM`)**: Turns the robot left by reducing the left motor's speed.
- **Right Turn (`R + PWM`)**: Turns the robot right by reducing the right motor's speed.
- **Stop (`S`)**: Stops the robot.

### Key Shortcuts
- Press `q` to quit both the color detection script (`main.py`) and the calibration tool (`threshold.py`).

## Troubleshooting
- **No Camera Detected**: Ensure that your camera is properly connected and that `cv2.VideoCapture(0)` works.
- **Serial Communication Issues**: Verify that the correct serial port is being used and that the Arduino is properly connected.
- **Robot Not Moving**: Check the motor wiring and make sure the motor driver is properly connected to the Arduino.
