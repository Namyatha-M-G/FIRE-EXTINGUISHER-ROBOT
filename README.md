# Automatic Fire Extinguisher Robot

## Overview

The Automatic Fire Extinguisher Robot is an embedded and IoT-based robotic system designed to detect fire-related conditions using sensor nodes and navigate toward the affected location for fire detection and extinguishing.

The system combines:

- ESP32-based sensor nodes
- Wireless communication
- Flame detection
- Motor control
- Servo positioning
- Relay control
- Water pump
- Web-based dashboard

---


## System Architecture

## System Architecture

```text
+---------------------+
|    Sensor Node      |
|---------------------|
| ESP32               |
| Flame Sensor        |
| MQ-2 Gas Sensor     |
+----------+----------+
           |
           | Wi-Fi + HTTP
           v
+---------------------+
|    Robot ESP32      |
|---------------------|
| Node Identification|
| Position Estimation |
| Navigation FSM      |
+----------+----------+
           |
   +-------+-------+
   |               |
   v               v
+---------+   +-------------+
| Motor   |   | Flame       |
| Control |   | Sensors     |
+---------+   +-------------+
      |             |
      +------+------+ 
             |
             v
      +-------------+
      | Servo Motor |
      +-------------+
             |
             v
      +-------------+
      | Relay Module|
      +-------------+
             |
             v
      +-------------+
      | Water Pump  |
      +-------------+
             |
             v
      +-------------+
      | Fire        |
      |Extinguishing|
      +-------------+
```


```text
Sensor Node
    │
    │ Wi-Fi + HTTP
    ▼
Robot ESP32
    │
    ├── Node Identification
    ├── Position Estimation
    ├── Navigation FSM
    │
    ├── Motor Control
    ├── Flame Sensors
    ├── Servo
    └── Relay + Pump
             │
             ▼
       Fire Extinguishing
```

---

## How It Works

1. Sensor nodes monitor their surroundings for fire-related conditions.

2. The sensor node reads sensor values using an ESP32.

3. The sensor information and node ID are communicated to the robot through Wi-Fi and HTTP.

4. The robot identifies the node that generated the alert.

5. Each node has a configured position represented using X-Y coordinates.

6. The robot estimates its position using dead reckoning based on movement speed, heading, turn rate, and elapsed time.

7. The navigation logic determines the direction and distance toward the target node.

8. The robot moves toward the target using the motor-control system.

9. After reaching the approximate node location, onboard flame sensing is used to confirm and locate the fire.

10. The servo positions the extinguishing mechanism.

11. The relay activates the water pump.

12. The robot verifies the fire condition after the extinguishing operation.

---

## Main Components

### Sensor Node

- ESP32
- Flame Sensor
- MQ-2 Gas Sensor
- Wi-Fi Communication
- HTTP Communication

### Robot

- ESP32
- Motor Driver
- DC Motors
- Flame Sensors
- Servo Motor
- Relay Module
- Water Pump
- ESP32-CAM Interface

---

## Communication

The system uses:

- Wi-Fi for wireless communication
- HTTP for communication between the sensor node, robot, and web interface

### Sensor Node Information

- Node ID
- Flame Status
- Gas Sensor Reading

The robot contains an HTTP server that receives information and commands and provides robot status to the frontend.

---

## Robot Navigation

The robot uses coordinate-based navigation with dead-reckoning position estimation.

### Position Estimation

```text
x = x + speed × cos(heading) × dt

y = y + speed × sin(heading) × dt
```

The robot also estimates its heading using the calibrated turning rate.

Each sensor node has a configured X-Y position. The robot uses the node ID received with an alert to identify the corresponding target position.

The robot does not have an exact absolute position. Instead, its position is estimated from its movement. A configured distance tolerance is used to determine when the robot has reached the target area.

---

## Robot Control

The robot uses a Finite State Machine (FSM) to manage its operating states.

### FSM States

- IDLE
- SCAN
- NAVIGATE
- FORWARD
- TURN LEFT
- TURN RIGHT
- SEARCH FIRE
- EXTINGUISH
- VERIFY
- RETURN HOME

The FSM determines the robot's next action based on sensor information and navigation status.

---

## Motor Control

The robot uses GPIO signals for motor direction and PWM for motor speed control.

```text
Direction GPIOs → Motor Direction
PWM             → Motor Speed
```

This allows the robot to:

- Move Forward
- Reverse
- Turn Left
- Turn Right
- Stop

---

## Fire Extinguishing

The general extinguishing sequence is:

```text
Flame Detection
      ↓
Fire Direction
      ↓
Servo Positioning
      ↓
Relay Activation
      ↓
Water Pump
      ↓
Fire Extinguishing
      ↓
Verification
```

---

## Web Dashboard

A web-based dashboard is included for monitoring and controlling the robot.

### Dashboard Features

- Robot Status
- Robot Position
- Heading
- Sensor-Node Alerts
- Flame Status
- Gas Readings
- Robot State
- Motor Status
- Pump Status

The frontend communicates with the robot through its HTTP server.

---

## Technologies Used

### Programming Language

- C

### Hardware Platform

- ESP32

### Development Framework

- ESP-IDF
- FreeRTOS

### Communication

- Wi-Fi
- HTTP
- UART

### Interfaces

- GPIO
- ADC
- PWM

### Frontend Technologies

- HTML
- CSS
- JavaScript

---

## My Contribution

My main contribution to the project was in frontend development, sensor and component integration, and system testing.

### Frontend Development

- Developed and integrated the web-based dashboard.
- Connected the frontend with the ESP32 server.

### Hardware Integration

- Integrated the flame sensor with the ESP32-based robot.
- Integrated the relay module for pump control.
- Integrated the servo motor for extinguisher positioning.
- Assisted in Wi-Fi server-related integration.

### Testing and Validation

- Tested individual sensors and components to verify proper operation.
- Performed integration testing after combining hardware modules.
- Supported physical assembly and testing of the robot prototype.
- Verified the behavior of integrated components during system-level testing.

My work primarily focused on hardware integration, sensor validation, frontend development, and testing of the complete system.

---

## Limitations

- The robot's position is estimated using dead reckoning rather than an absolute positioning system.
- Position error can accumulate due to:
  - Wheel slip
  - Motor variation
  - Battery level changes
  - Surface friction
- Node coordinates must be configured correctly for accurate navigation.

---

## Future Improvements

- Wheel encoders for improved odometry
- IMU-based orientation correction
- Sensor fusion for improved position estimation
- More robust wireless communication
- Improved obstacle detection and avoidance
- Improved fire detection
- Remote monitoring and data logging

---

## Project Status

Working academic prototype developed as an embedded and IoT-based fire detection and extinguishing project.
