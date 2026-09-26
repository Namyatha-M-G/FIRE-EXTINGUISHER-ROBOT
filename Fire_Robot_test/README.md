# Fire Fighting Robot Firmware

ESP32 fire fighting robot controller firmware built using ESP-IDF and FreeRTOS.

## Overview

The controller firmware coordinates localized target scanning, autonomous tracking, coordinate-based path navigation, and manual overrides.

Key components:
- `main/main.c` - FreeRTOS control loops, coordinate tracking, autonomous state transitions, and Return-to-Home.
- `main/motor.c` - L298N motor driver control, turning torque boost, and speed setting limits.
- `main/servo.c` - Sweeping nozzle servo commands.
- `main/flame.c` - Local flame sensor array reads.
- `main/relay.c` - Water pump relay triggers.
- `main/wifi_server.c` - Web server hosting status JSON, control commands, OTA updating, and the embedded Dashboard interface.

---

## Odometer Calibration & Training

The robot computes its current position `(x, y, heading)` using a time-integrated dead reckoning model. Because surfaces (concrete, asphalt, tile, carpet) have different frictions, you can train/calibrate the odometry parameters directly from the **Laptop Dashboard** without re-compiling:

1. **Travel Speed (m/s)**: Physical distance traveled per second going straight.
2. **Turn Rate (rad/s)**: Physical angle rotated per second during turns.

*These parameters are saved in NVS memory and persist across power cycles.*

To change the default parameters inside the code, edit:
[main.c](file:///D:/Client%20Projects/Fire%20Exitinguisher/Fire_robot_test/Fire_Robot_test/main/main.c)

```c
static float s_speed_m_s = 0.20f;  // Default forward speed limit (m/s)
static float s_turn_rad_s = 1.50f; // Default turning rate (rad/s)
```

---

## Customizing Speeds & Steering Boost

- **Manual Speed Limit**: Can be adjusted dynamically via the dashboard slider.
- **Skid-Steering Turn Torque Boost**: Turning (skid steering) on high-friction petrol bunk surfaces or carpet requires significant torque. The firmware automatically boosts turning speed to a minimum of **85%** in manual mode to prevent wheel stall, regardless of what the speed slider is set to.

To modify this boost threshold, navigate to:
[motor.c](file:///D:/Client%20Projects/Fire%20Exitinguisher/Fire_robot_test/Fire_Robot_test/main/motor.c)

```c
void motor_left(void) {
    ...
    if (turn_speed < 85) {
        turn_speed = 85; // Boost turning speed threshold
    }
    ...
}
```

---

## Pin Map

| Device | ESP32 Pin |
| --- | --- |
| Motor IN1 | GPIO14 |
| Motor IN2 | GPIO27 |
| Motor IN3 | GPIO26 |
| Motor IN4 | GPIO25 |
| ENA PWM | GPIO33 |
| ENB PWM | GPIO32 |
| Servo | GPIO13 |
| Relay | GPIO23 |
| Flame Front | GPIO34 |
| Flame Left | GPIO35 |
| Flame Right | GPIO39 |
| ESP32-CAM TX | GPIO16 |
| ESP32-CAM RX | GPIO17 |

---

## Wi-Fi Control

The robot spawns a local access point:
- **SSID**: `FireRobot`
- **Password**: `fire12345`
- **Dashboard URL**: `http://192.168.4.1/`

---

## Build and Flash

1. Open your ESP-IDF command prompt and navigate to the directory:
   ```powershell
   cd "D:\Client Projects\Fire Exitinguisher\Fire_robot_test\Fire_Robot_test"
   ```
2. **Build**:
   ```powershell
   idf.py build
   ```
3. **Flash**:
   ```powershell
   idf.py -p COMx flash monitor
   ```
   *(Replace `COMx` with your ESP32 serial port).*
