# ObstacleAvoidance-C
Robot Obstacle Avoidance

## Author
Nathan Shorez

---

## Project Overview
This project implements an obstacle avoidance and line-following system using C++ and Arduino. The system leverages sonar sensors for detecting obstacles and a PD controller for maintaining alignment with the line. The primary objective is to navigate a predefined path while avoiding collisions with detected obstacles.

---

## Key Components
- **Line Following:** Utilizes a PD controller to maintain path alignment based on sensor feedback.
- **Obstacle Avoidance:** Integrates sonar sensors to detect and avoid obstacles in the robot’s path.

---

## Hardware/Software Requirements
- Arduino-compatible microcontroller
- Sonar sensors for obstacle detection
- IR sensors for line following
- C++ compiler (e.g., g++)
- Arduino IDE for uploading and monitoring

---

## Compilation and Execution
1. **Navigate to the src directory:**
   ```bash
   cd src/
   ```
2. **Compile the source files:**
   ```bash
   g++ main.cpp PDcontroller.cpp sonar.cpp -o obstacle_avoidance
   ```
3. **Upload to Arduino:**
   - Open the Arduino IDE
   - Load `main.cpp`
   - Select the appropriate COM port and board
   - Upload the sketch

---

## Sample Output
Upon running, the system will output distance readings from sonar sensors and PD controller adjustments based on line following data. Example:
```
Obstacle detected at 15cm. Adjusting path.
Line deviation: 0.35
PD Output: -25
```
---

## Dependencies
- Arduino IDE
- C++ Compiler (e.g., g++)
- IR Sensors (for line following)
- Sonar Sensors (for obstacle detection)

---
