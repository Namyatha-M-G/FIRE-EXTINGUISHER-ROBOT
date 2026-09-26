# Room Sensor Node (NodeMCU / ESP32)

Distributed room monitor nodes audit the local environment for smoke/gas (using MQ-2) and flame anomalies, sending HTTP post alerts to the central robot.

## Key Configurations

- **Node Identity**: Each room sensor node must have a unique ID. To change the node ID, navigate to:
  [main.c](file:///D:/Client%20Projects/Fire%20Exitinguisher/Fire_robot_test/Sensor_Node_NodeMCU/main/main.c)

  Update the `NODE_ID` macro on line 19:
  ```c
  #define NODE_ID 1 // Change to 2 for the second room node, etc.
  ```

- **Wi-Fi Credentials**: Configured to connect to the robot's Access Point:
  ```c
  #define WIFI_SSID "FireRobot"
  #define WIFI_PASS "fire12345"
  ```

- **Pin Assignments**:
  - **Flame Sensor Pin**: `GPIO14` (Digital Input, Active-Low)
  - **MQ-2 Gas Sensor Pin**: `GPIO34` (Analog Input read via ADC1 Channel 6)

## Customizing Anomaly Trigger Thresholds

The thresholds that classify a reading as an anomaly are processed by the central robot. To change the gas threshold (default is `400`), modify the threshold value in the robot firmware's main code:
[main.c](file:///D:/Client%20Projects/Fire%20Exitinguisher/Fire_robot_test/Fire_Robot_test/main/main.c)

Modify `set_node_alert` around line 68:
```c
void set_node_alert(int node_id, bool has_flame, int gas_level) {
    ...
    // Alerts trigger if flame is detected or if gas level climbs above 400 ppm
    s_nodes[node_id - 1].active_alert = has_flame || (gas_level > 400); 
    ...
}
```

## Compilation and Flashing

Open your ESP-IDF command prompt and navigate to the directory:
```powershell
cd "D:\Client Projects\Fire Exitinguisher\Fire_robot_test\Sensor_Node_NodeMCU"
```

1. **Build**:
   ```powershell
   idf.py build
   ```
2. **Flash**:
   ```powershell
   idf.py -p COMx flash monitor
   ```
   *(Replace `COMx` with your NodeMCU board port).*
