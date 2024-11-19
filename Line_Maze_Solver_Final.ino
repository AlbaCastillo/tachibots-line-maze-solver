// Line Maze Solver (LSRB Algorithm)
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Motors
AF_DCMotor motori(1);  //Left motor - connected to terminal 1
AF_DCMotor motord(2);  //Right motor - connected to terminal 2

// Speed Constants
const int NORMAL_SPEED = 80;
const int HIGHER_SPEED = 120;
const int LOWER_SPEED = 40;
const float SPEED_INCREMENT = 0.2;
const int TURN_HIGHER_SPEED = 120;
const int TURN_LOWER_SPEED = 40;
const float TURN_SPEED_INCREMENT = 0.2;

// Reflectance Sensors TCRT5000
// Sensor arrays
const uint8_t SensorCountSetup = 2;  // Sensors initialized in setup
const uint8_t SensorCountPost = 4;  // Sensors initialized after motors start
const uint8_t SensorCountTotal = SensorCountSetup + SensorCountPost;  // Total sensors

//Sensor position
//  D2(0)      -D6(2)-     D10(4)
// -D9(1)-     -D9(3)-    -D12(5)-
// D3(1), D4(2), D5(3), D7(5), For turn control
// D2(0) and D6(4) For goal control


// Sensor arrays digital pins
// const int pinIRdSetup[SensorCountSetup] = {10, 9};            // Pins configured in setup
// const int pinIRdPost[SensorCountPost] = {6, 0, 2, 2};      // Pins configured post-start
// const int pinIRdPost[SensorCountPost] = {6, 0, 2, 12};      // Pins configured post-start

// TODO: Sensor arrays analog pins
const int pinIRdSetup[SensorCountSetup] = {A3, A2};       // Pins configured in setup
const int pinIRdPost[SensorCountPost] = {A5, A4, A1, A0}; // Pins configured post-start

int pinIRd[SensorCountTotal];                                // Pines para los sensores infrarrojos
const int THRESHOLD = 469; // For a single threshold
// const int thresholds[SensorCountTotal] = {500, 500, 500, 505, 495, 500}; // For individual thresholds per sensor:

int IRvalueSetup[SensorCountSetup] = {0, 0};            // Sensor values for setup
int IRvaluePost[SensorCountPost] = {0, 0, 0, 0};        // Sensor values for post-start
int IRvalue[SensorCountTotal];                          // Values readed by sensors - high(on-black) or low(off-white)

bool calibrated = false;  // Tracks whether calibration is complete
bool postSensorsInitialized = false;  // Tracks whether post sensors are initialized

// Path Variables
String path;          // Path recorder
String shortestPath;  // Records the shortest path after optimization
int goal = 0;         // find the goal? 0=no - 1=yes

// For calculate the degrees of the turn
Adafruit_MPU6050 mpu;   // Gyroscope readings from Adafruit MPU6050
unsigned long lastTime;
float deltaTime;
float angle = 0;        // Almacena el ángulo acumulado

// Function Prototypes
void setup();
void loop();
void front();
void turn(float targetAngle);
void updateGyro();
void back();
void stop();
void simplifyPath();
void gbontrackR();
void gbontrackL();
void go(String path);
void lineValue(const int *pins, int *values, uint8_t count);
void debugInfraRed(const int *values, uint8_t count, const char *label);
void performCalibration();
void initializePostSensors();

// Functions
void setup() {
  Serial.begin(9600);
  Serial.println("Setup");

  //Configurando motores
  motori.setSpeed(NORMAL_SPEED);
  motord.setSpeed(NORMAL_SPEED);
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(100);

  // MPU Configuration
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050");
  // }
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  lastTime = millis();

  // Configure pins for setup sensors
  for (int i = 0; i < SensorCountSetup; i++) {
    pinMode(pinIRdSetup[i], INPUT_PULLUP);  // Configura cada pin como entrada
  }

  delay(500);
}

void loop() {
  if (!calibrated) {
    performCalibration();
  } else {

    // Initialize post sensors after motors start
    if (!postSensorsInitialized) {
      initializePostSensors();
    }

    // Read Values
    lineValue(pinIRd, IRvalue, SensorCountTotal);
    Serial.println("Loop");
    debugInfraRed(IRvalue, SensorCountTotal, "All Sensor States");
    delay(500);

    /* Get new sensor events with the readings */
    // sensors_event_t a, g, temp;
    // mpu.getEvent(&a, &g, &temp);

    //
    if (goal == 1) {
      if (!IRvalue[0] && !IRvalue[1] && IRvalue[2] && IRvalue[3] && !IRvalue[4] && !IRvalue[5]) {  // Positioned at the start line
        go(shortestPath);                                                                          //
      }
      stop();
    }

    //FORWARD Condition [001100] (Go straight "S")
    if (!IRvalue[0] && !IRvalue[1] && IRvalue[2]) {
    // if ((!IRvalue[0] && !IRvalue[1] && IRvalue[2] && IRvalue[3]) || (!IRvalue[0] && !IRvalue[1] && IRvalue[2] && IRvalue[3])) {
      Serial.println("Move FORWARD");
      path += 'S';
      front();
    }

    //Left Turn Condition [010100] ("L")
    else if ((IRvalue[1] && IRvalue[3]) || (IRvalue[0] && IRvalue[2])) {
      Serial.println("Turn LEFT");
      path += 'L';
      turn('L');
    }

    //Right Turn Condition [000101] ("R")
    // else if (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && IRvalue[3] && !IRvalue[4] && IRvalue[5]) {
    else if ((IRvalue[5] && IRvalue[3]) || (IRvalue[4] && IRvalue[2])) {
      Serial.println("Turn RIGHT");
      path += 'R';
      turn('R');
    }

    //U Turn Condition [000100] ("U")
    else if (!IRvalue[3] && (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[4] && !IRvalue[5])) {
      Serial.println("U-TURN");
      path += 'U';
      turn('U');
    }

    //Stop Condition [111111] Final of the maze
    else if (IRvalue[0] && IRvalue[1] && IRvalue[2] && IRvalue[3] && IRvalue[4] && IRvalue[5]) {
      Serial.println("Stop");
      stop();
      delay(500);
    }

    //Go a litle Back if [000000] to try to get back on track
    if (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5]) {
      Serial.println("Go BACK");
      back();
    }

    //When there are no turns but the road is not straight, curvatures with a minimum radius of 30cm
    // [000001] turn slowly to the right until [000100] (While continuing to move forward, only slowing down on a single motor)
    if ((!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && IRvalue[5]) || (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && IRvalue[4])) {
      gbontrackR();  //correct to the right
    }

    //[010000] turn slowly to the left until [000100] (While continuing to move forward, only slowing down on a single motor)
    if ((IRvalue[1] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5]) || (IRvalue[0] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5])) {
      gbontrackL();  //correct to the left
    }

    simplifyPath();
  }
}

void front() {
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);

  // Proposed condition (IRvalue[2] && IRvalue[3] && !IRvalue[0] && !IRvalue[1] && !IRvalue[4] && !IRvalue[5])
  // The robot moves forward while the middle sensors detect the line, and side sensors detect no line.
  while (!((IRvalue[3]) && (IRvalue[1] || IRvalue[5]))) {
    Serial.print("while (FORWARD)");
    motori.run(FORWARD);
    motord.run(FORWARD);
    lineValue(pinIRd, IRvalue, SensorCountTotal);
    delay(100);

    // Correct to the right if necessary => [000001] turn slowly to the right until [000100]
    if ((!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && IRvalue[5]) || (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && IRvalue[4])) {
      gbontrackR();  // correct to the right
    }
    // Correct to the left if necessary => [010000] turn slowly to the left until [000100]
    else if ((IRvalue[1] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5]) || (IRvalue[0] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5])) {
      gbontrackL();  //correct to the left
    }
    //Go a litle Back if [000000] to try to get back on track
    else if (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5]) {
      back();
    }
    lineValue(pinIRd, IRvalue, SensorCountTotal);
  }
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
}


void turn(char direction) {
  // Stop the motors before turning
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);

  Serial.print("Turning ");
  Serial.println(direction);

  // Variables for motor speeds
  float leftSpeed = NORMAL_SPEED;
  float rightSpeed = NORMAL_SPEED;

  // Determine initial motor directions
  int leftDirection = FORWARD;
  int rightDirection = FORWARD;

  // Adjust motor directions based on the turn direction
  if (direction == 'L') { // Left turn
    // Both motors move forward
    leftDirection = FORWARD;
    rightDirection = FORWARD;
    leftSpeed = TURN_LOWER_SPEED;
    rightSpeed = TURN_HIGHER_SPEED;
  } else if (direction == 'R') { // Right turn
    // Both motors move forward
    leftDirection = FORWARD;
    rightDirection = FORWARD;
    leftSpeed = TURN_HIGHER_SPEED;
    rightSpeed = TURN_LOWER_SPEED;
  } else if (direction == 'U') { // U-turn
    // Left motor backward, right motor forward
    leftDirection = BACKWARD;
    rightDirection = FORWARD;
  } else {
    Serial.println("Invalid direction!");
    return;
  }

  // Gradually adjust speeds during the turn
  while (!IRvalue[3] || !IRvalue[2]) {
    // Read sensor values
    lineValue(pinIRd, IRvalue, SensorCountTotal);

    // Exit condition: line detected by the center-front sensor
    if (IRvalue[2] == 1) {
      break;
    }

    // Adjust speeds incrementally
    if (direction == 'L') {
      // Turn left by decreasing left motor speed, increasing right motor speed
      leftSpeed -= TURN_SPEED_INCREMENT; // Decrease left speed
      rightSpeed += TURN_SPEED_INCREMENT; // Increase right speed
    } else if (direction == 'R') {
      // Turn right by increasing left motor speed, decreasing right motor speed
      leftSpeed += TURN_SPEED_INCREMENT; // Increase left speed
      rightSpeed -= TURN_SPEED_INCREMENT; // Decrease right speed
    } else if (direction == 'U') {
      // For U-turns, both motors run at higher speeds, opposite directions
      leftSpeed += TURN_SPEED_INCREMENT;
      rightSpeed += TURN_SPEED_INCREMENT;
    }

    // Apply motor speeds and directions using the new function
    setMotorSpeeds(leftDirection, rightDirection, leftSpeed, rightSpeed);

    // Debugging: Print current speeds
    Serial.print("Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.println(rightSpeed);

    delay(10); // Stability delay
  }

  // Stop the motors after completing the turn
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);

  // Reset motor speeds to normal
  setMotorSpeeds(FORWARD, FORWARD, NORMAL_SPEED, NORMAL_SPEED);
}

// Function to set motor speeds and directions
void setMotorSpeeds(int leftDirection, int rightDirection, float leftSpeed, float rightSpeed) {
  motori.setSpeed(leftSpeed);
  motord.setSpeed(rightSpeed);
  motori.run(leftDirection);
  motord.run(rightDirection);
}


void updateGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  //TODO: Chequear funcionamiento cuando tenga que girar a la derecha (angulos negativos)
  float gyroZ = g.gyro.z * (180 / PI);  // rad/s a deg/s
  angle += gyroZ * deltaTime;
}


void back() {
  motori.run(BACKWARD);
  motord.run(BACKWARD);
  delay(20);
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
}


void stop() {
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(50);
}


// Function to optimize the path by simplifying it
void simplifyPath() {
  // Remove unnecessary backtracking (LUR = U / LUS = R / RUL = U / SUL = R / SUS = U / LUL = S)
  // Path simplification using LSRB conditions
  shortestPath = path;

  shortestPath.replace("LUR", "U");
  shortestPath.replace("LUS", "R");
  shortestPath.replace("RUL", "U");
  shortestPath.replace("SUL", "R");
  shortestPath.replace("SUS", "U");
  shortestPath.replace("LUL", "S");

  // Serial.print("Optimized Path: ");
  Serial.println(shortestPath);
}


void gbontrackR() {  //while is not [001100] do this
  lineValue(pinIRd, IRvalue, SensorCountTotal);
  Serial.println("Adjusting to the right");
  float higherSpeed = HIGHER_SPEED;
  float lowerSpeed = LOWER_SPEED;
  motori.setSpeed(HIGHER_SPEED);
  motord.setSpeed(LOWER_SPEED);

  while (!IRvalue[3] || !IRvalue[2]) {
    Serial.print("HighSpeed: ");
    Serial.print(higherSpeed);
    Serial.print(" | LowSpeed: ");
    Serial.println(lowerSpeed);

    motori.setSpeed(higherSpeed);
    motord.setSpeed(lowerSpeed);
    motori.run(FORWARD);
    motord.run(FORWARD);
    
    delay(10);
    lineValue(pinIRd, IRvalue, SensorCountTotal);

    higherSpeed += SPEED_INCREMENT;
    lowerSpeed -= SPEED_INCREMENT;
  }

  // Reset motor speeds
  motori.setSpeed(NORMAL_SPEED);
  motord.setSpeed(NORMAL_SPEED);
  delay(10);
}


void gbontrackL() {  //while is not [001100] do this
  lineValue(pinIRd, IRvalue, SensorCountTotal);
  Serial.println("Adjusting to the left");
  motori.setSpeed(LOWER_SPEED);
  motord.setSpeed(HIGHER_SPEED);
  float higherSpeed = HIGHER_SPEED;
  float lowerSpeed = LOWER_SPEED;

  // Proposed Condition: (!IRvalue[2] || !IRvalue[3]) 
  while (!IRvalue[3] || !IRvalue[2]) {
    Serial.print("HighSpeed: ");
    Serial.print(higherSpeed);
    Serial.print(" | LowSpeed: ");
    Serial.println(lowerSpeed);

    motori.setSpeed(lowerSpeed);
    motord.setSpeed(higherSpeed);
    motori.run(FORWARD);
    motord.run(FORWARD);

    delay(10);
    lineValue(pinIRd, IRvalue, SensorCountTotal);

    higherSpeed +=0.2;
    lowerSpeed -=0.2;
  }

  // Reset motor speeds
  motori.setSpeed(NORMAL_SPEED);
  motord.setSpeed(NORMAL_SPEED);
  delay(10);
}


void go(String path) {  // Follow the recorded path
  for (int i = 0; i < path.length(); i++) {
    char step = path[i];
    if (step == 'S') front();
    else if (step == 'L') turn('L');
    else if (step == 'R') turn('R');
    else if (step == 'U') turn('U');
  }
  stop();
}


// Digital Version
// void lineValue(const int *pins, int *values, uint8_t count) {
//   for (int i = 0; i < count; i++) {
//     values[i] = digitalRead(pins[i]);
//   }
//   delay(10);
// }


// TODO: Analog Version

void lineValue(const int *pins, int *values, uint8_t count) {
  for (int i = 0; i < count; i++) {
    int sensorValue = analogRead(pins[i]);
    if (sensorValue > THRESHOLD) {
      values[i] = 1; // Black line detected
    } else {
      values[i] = 0; // White surface detected
    }
    // Using individual thresholds:
    // if (sensorValue < thresholds[i]) {
    //   values[i] = 1; // Black line detected
    // } else {
    //   values[i] = 0; // White surface detected
    // }
  }
  delay(10);
}



// Function to print sensor debug messages
void debugInfraRed(const int *values, uint8_t count, const char *label) {
  Serial.print(label);
  Serial.print(": ");
  for (int i = 0; i < count; i++) {
    Serial.print(values[i]);
    Serial.print(" ");
  }
  Serial.println();
}


// Function to perform calibration
void performCalibration() {
  Serial.println("Calibrating...");

  // Check if all setup sensors are in air (all HIGH)
  bool allSensorsHigh = true;
  lineValue(pinIRdSetup, IRvalueSetup, SensorCountSetup);

  for (int i = 0; i < SensorCountSetup; i++) {
    if (IRvalueSetup[i] == LOW) {
      allSensorsHigh = false;
    }
  }

  debugInfraRed(IRvalueSetup, SensorCountSetup, "Setup Sensor States");

  if (allSensorsHigh) {
    Serial.println("All setup sensors detect air. Waiting for black signal...");

    // Wait until any setup sensor detects black (LOW)
    bool blackDetected = false;
    while (!blackDetected) {
      lineValue(pinIRdSetup, IRvalueSetup, SensorCountSetup);
      for (int i = 0; i < SensorCountSetup; i++) {
        if (IRvalueSetup[i] == LOW) {
          blackDetected = true;
          break;
        }
      }
      delay(100);  // Short delay for stability
    }

    Serial.println("Black detected! Starting in 2 seconds...");
    delay(2000);  // Wait 2 seconds
    calibrated = true;

    // Start motors
    motori.run(FORWARD);
    motord.run(FORWARD);
  }

  delay(100);  // Small delay for stability
}


// Function to initialize post sensors and the combined array
void initializePostSensors() {
  Serial.println("Initializing post sensors...");
  
  // Initialize post sensors
  for (int i = 0; i < SensorCountPost; i++) {
    pinMode(pinIRdPost[i], INPUT_PULLUP);
  }

  pinIRd[0] = pinIRdPost[0];
  pinIRd[1] = pinIRdPost[1];
  pinIRd[2] = pinIRdSetup[0];
  pinIRd[3] = pinIRdSetup[1];
  pinIRd[4] = pinIRdPost[2];
  pinIRd[5] = pinIRdPost[3];

  postSensorsInitialized = true;
  Serial.println("Post sensors initialized and combined array ready.");

  // Normal operation after calibration and post sensor initialization
  Serial.println("Calibrated and post sensors initialized. Monitoring sensors...");
}
