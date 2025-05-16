#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;

Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line .5
#define kd_line .5
#define kp_obs 10
#define kd_obs 5

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

// Sensor calibration
void calibrateSensors() {
  //TODO: write function to calibration IR line sensors
  motors.setSpeeds(-50, 50);  // Turn left
  for (int i = 0; i < 30; i++) {
    lineSensors.calibrate();  // Calibrate during left turn
    delay(20);
  }

  // Face back forward
  motors.setSpeeds(50, -50);
  delay(1500);

  // Stop
  motors.setSpeeds(0, 0);  // Stop the motors to face forward
  for (int i = 0; i < 10; i++) {
    lineSensors.calibrate();
    delay(20);
  }

  // Turn right 90 degrees
  motors.setSpeeds(50, -50);
  for (int i = 0; i < 30; i++) {
    lineSensors.calibrate();  // Calibrate during right turn
    delay(20);
  }

  // face back forward
  motors.setSpeeds(-50, 50);
  delay(1500);

  // Stop the robot after calibration
  motors.setSpeeds(0, 0);  // Stop the motors completely
  delay(1000);
}

// Initialization of robot parts
void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90);  // turn servo forward
  delay(2000);

  calibrateSensors();
}

// Loop function which swithes b/w follow line and avoid obstacle
void loop() {

  //TODO: main code here
  //Hint: may need additional functions

  // Original call to follow line without obstacle avoidance:
  // followLine();

  // Obstacle avoidance logic:
  float obstacleDistance = sonar.readDist(); // see if an obstacle is located in sonar range

  if (obstacleDistance < 10.0) { // if dist is less than 10
    avoidObstacle(); // avoid the obstacle
  } else {
    followLine(); // otherwise follow the line
  }
}

// Function to follow the line using PD controller
void followLine() {
  unsigned int lineSensorValues[5]; // array to hold sensor values for each of the 5 sensors

  // Get position of black line from 0 to 4000
  int position = lineSensors.readLineBlack(lineSensorValues, LineSensorsReadMode::On);

  // Calculate error
  int error = position - 2000;

  // Get PD controller output for line following
  double correction = pd_line.update(error, 0);  // 0 is the setpoint for position - 2000 (we want them equal)

  int leftSpeed = baseSpeed + correction;   // adjust left wheel speed
  int rightSpeed = baseSpeed - correction;  // adjust right wheel speed

  // Set motor speeds
  motors.setSpeeds(leftSpeed, rightSpeed);
}

// Function to avoid obstacle and return to line when detected again
void avoidObstacle() {
  // Array for line sensor values
  unsigned int lineSensorValues[5];

  // Stop the robot
  motors.setSpeeds(0, 0);
  // Turn away from the obstacle to the right
  motors.setSpeeds(50, -50);
  delay(1700);
  motors.setSpeeds(0, 0);
  delay(500);

  // turn servo to the left:
  servo.write(180);
  delay(1000);

  // Move forward slowly (so errors can be read without making drastic adjustments)
  motors.setSpeeds(baseSpeed, baseSpeed);
  delay(1000);

  while (true) {
    // get the control signal for avoiding the obstacle 
    double PDsignal = pd_obs.update(sonar.readDist(), 10.0);

    // Update left and right wheel speeds based on error
    double speed_left = baseSpeed - (PDsignal);
    double speed_right = baseSpeed + PDsignal;

    // Set the motor to the updated wheel speeds
    motors.setSpeeds(speed_left, speed_right);
    delay(40);

    // Read the light sensors signals into the light sensor value array
    int position = lineSensors.readLineBlack(lineSensorValues, LineSensorsReadMode::On);

    // motors.setSpeeds(10, 10);

    Serial.println(position);

    // Check if any sensors return values that suggest black tape is under them
    if ((position > 0) && (position < 4000)) {

      //May need to turn the robot to the right again:
      motors.setSpeeds(0, 0);
      delay(40);
      motors.setSpeeds(50, 50);
      delay(1000);
      motors.setSpeeds(50, -50);
      delay(2000);
      motors.setSpeeds(0,0);
      delay(40);

      // Turn servo forward again (to detect obstacles in front of it)
      servo.write(90);
      delay(40);
      // Return to the follow line function
      followLine();
      // Break out of the while loop
      break;
    }
  }
}

