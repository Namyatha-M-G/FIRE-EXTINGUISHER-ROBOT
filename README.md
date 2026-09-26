Automatic Fire Extinguisher Robot

Overview

The Automatic Fire Extinguisher Robot is an embedded and IoT-based robotic system designed to detect fire-related conditions using sensor nodes and navigate toward the affected location for fire detection and extinguishing.

The system combines ESP32-based sensor nodes, wireless communication, flame detection, motor control, servo positioning, relay control, a water pump, and a web-based dashboard.

System Architecture
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
       
How It Works
Sensor nodes monitor their surroundings for fire-related conditions.

The sensor node reads sensor values using an ESP32.

The sensor information and node ID are communicated to the robot through Wi-Fi and HTTP.

The robot identifies the node that generated the alert.

Each node has a configured position represented using X-Y coordinates.

The robot estimates its position using dead-reckoning based on movement speed, heading, turn rate, and elapsed time.

The navigation logic determines the direction and distance toward the target node.

The robot moves toward the target using the motor-control system.

After reaching the approximate node location, onboard flame sensing is used to confirm and locate the fire.

The servo positions the extinguishing mechanism.

The relay activates the water pump.

The robot verifies the fire condition after the extinguishing operation.

Main Components
Sensor Node
ESP32

Flame Sensor

MQ-2 Gas Sensor

Wi-Fi communication

HTTP communication

Robot
ESP32

Motor driver

DC motors

Flame sensors

Servo motor

Relay module

Water pump

ESP32-CAM interface

Communication
The system uses:

Wi-Fi for wireless communication

HTTP for communication between the sensor node, robot and web interface

The sensor node can provide information such as:

Node ID
Flame status
Gas sensor reading
The robot contains an HTTP server that receives information and commands and provides robot status to the frontend.

Robot Navigation
The robot uses coordinate-based navigation with dead-reckoning position estimation.

The estimated position is updated using:

x = x + speed × cos(heading) × dt

y = y + speed × sin(heading) × dt
The robot also estimates its heading using the calibrated turning rate.

Each sensor node has a configured X-Y position. The robot uses the node ID received with an alert to identify the corresponding target position.

The robot does not have an exact absolute position. Instead, its position is estimated from its movement. A configured distance tolerance is used to determine when the robot has reached the target area.

Robot Control
The robot uses a Finite State Machine (FSM) to manage its operating states.

Example states include:

IDLE
SCAN
NAVIGATE
FORWARD
TURN LEFT
TURN RIGHT
SEARCH FIRE
EXTINGUISH
VERIFY
RETURN HOME
The FSM determines the robot's next action based on sensor information and navigation status.

Motor Control
The robot uses GPIO signals for motor direction and PWM for motor speed control.

Direction GPIOs → Motor direction
PWM             → Motor speed
This allows the robot to move forward, reverse, turn left, turn right, and stop.

Fire Extinguishing
The general extinguishing sequence is:

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
Web Dashboard
A web-based dashboard is included for monitoring and controlling the robot.

The dashboard can display information such as:

Robot status

Robot position

Heading

Sensor-node alerts

Flame status

Gas readings

Robot state

Motor status

Pump status

The frontend communicates with the robot through its HTTP server.

Technologies Used
C

ESP32

ESP-IDF

GPIO

ADC

PWM

UART

Wi-Fi

HTTP

FreeRTOS

HTML

CSS

JavaScript

My Contribution
My main contribution to the project was in frontend development, sensor/component integration, and system testing.

I worked on:

Developing and integrating the web-based frontend/dashboard.

Connecting and integrating the flame sensor, relay, and servo components with the ESP32-based robot.

Working with the Wi-Fi server-related integration.

Testing individual sensors and components to verify that they were functioning correctly.

Testing the components again after integrating them with the other parts of the robot.

Supporting the physical integration and testing of the working robot model.

Verifying the behaviour of the integrated components during system testing.

My work primarily focused on hardware integration, sensor validation, frontend development, and testing of the complete system.

Limitations
The robot's position is estimated using dead reckoning rather than an absolute positioning system. Therefore, position error can accumulate due to factors such as wheel slip, motor variation, battery level, and surface friction.

Node coordinates also need to be configured correctly for the robot to navigate toward the intended locations.

## Future Improvements

The following improvements can be considered for future versions of the system:

- **Wheel Encoders:** Add wheel encoders to obtain more accurate wheel movement measurements and improve position estimation.
- **IMU Integration:** Use an IMU to improve heading estimation and reduce navigation errors.
- **Sensor Fusion:** Combine wheel encoder and IMU data to improve overall robot localization.
- **Obstacle Detection and Avoidance:** Add ultrasonic or LiDAR-based sensing so the robot can detect and avoid obstacles while navigating.
- **Improved Fire Detection:** Integrate camera-based fire detection to complement the existing flame sensors.
- **More Robust Communication:** Improve communication reliability between sensor nodes and the robot, including handling Wi-Fi disconnections.
- **Multiple Sensor Nodes:** Extend the system to support a larger number of distributed sensor nodes.
- **Remote Monitoring:** Add remote monitoring and data logging so sensor alerts and robot activity can be stored and reviewed.
- **Improved Navigation:** Develop more advanced path-planning and localization methods for environments with obstacles and changing layouts.
- **Power Management:** Introduce battery monitoring and low-power operation for longer deployment of sensor nodes and the robot.

  
Project Status
Working academic prototype developed as an embedded and IoT-based fire detection and extinguishing project.


