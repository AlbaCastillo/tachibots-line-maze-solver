// Line Maze Solver (LSRB Algorithm)

//Motors
#include <AFMotor.h>
AF_DCMotor motori(1);  //Left motor - connected to terminal 1
AF_DCMotor motord(2);  //Right motor - connected to terminal 2

// Reflectance Sensors TCRT5000
// Sensor arrays
const uint8_t SensorCountSetup = 2;  // Sensors initialized in setup
const uint8_t SensorCountPost = 4;  // Sensors initialized after motors start
const uint8_t SensorCountTotal = SensorCountSetup + SensorCountPost;  // Total sensors

const int pinIRdSetup[SensorCountSetup] = {6, 9};  // Pins configured in setup
const int pinIRdPost[SensorCountPost] = {0, 2, 10, 12};      // Pins configured post-start

int pinIRd[SensorCountTotal];                    // Pines para los sensores infrarrojos

int IRvalueSetup[SensorCountSetup] = {0, 0};  // Sensor values for setup
int IRvaluePost[SensorCountPost] = {0, 0, 0, 0};       // Sensor values for post-start
int IRvalue[SensorCountTotal];  // Valores leídos de los sensores - high (on - black) or low (off - white)

bool calibrated = false;  // Tracks whether calibration is complete
bool postSensorsInitialized = false;  // Tracks whether post sensors are initialized


// Path
String path;          // Path recorder
String shortestPath;  // Records the shortest path after optimization
int goal = 0;         // find the goal? 0=no - 1=yes

// Gyroscope readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// For calculate the degrees of the turn
float deltaTime;
unsigned long lastTime;
float angle = 0;        // Almacena el ángulo acumulado
float targetAngle = 0;  // Ángulo objetivo (90 o 180 grados)


// Ultrasonic Sensor
//const int Trigger = 2;   //Pin digital 2 para el Trigger del sensor
//const int Echo = 3;   //Pin digital 3 para el Echo del sensor

void setup() {
  Serial.begin(9600);
  Serial.println("Setup");

  //Configurando motores
  motori.setSpeed(100);
  motori.run(RELEASE);

  motord.setSpeed(100);
  motord.run(RELEASE);
  delay(100);

  // MPU Configuration
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

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

    if (!postSensorsInitialized) {
      // Initialize post sensors after motors start
      initializePostSensors();
      // Normal operation after calibration and post sensor initialization
      Serial.println("Calibrated and post sensors initialized. Monitoring sensors...");
    }

    // Read Values
    lineValue(pinIRd, IRvalue, SensorCountTotal);
    Serial.println("Loop");
    delay(500);

    // put your main code here, to run repeatedly:
    //Sensor position
    //  D2(0)      -D6(2)-     D10(4)
    // -D4(1)-     -D8(3)-    -D12(5)-

    // D3(1), D4(2), D5(3), D7(5), For turn control
    // D2(0) and D6(4) For goal control

    delay(7);  // To make sampling rate around 100hz
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //
    if (goal == 1) {
      if (!IRvalue[0] && !IRvalue[1] && IRvalue[2] && IRvalue[3] && !IRvalue[4] && !IRvalue[5]) {  // Positioned at the start line
        go(shortestPath);                                                                          //
      }
      stop();
    }

    debugInfraRed(IRvalue, SensorCountTotal, "All Sensor States");
    delay(500);

    //FORWARD Condition [001100] (Go straight "S")
    if ((!IRvalue[0] && !IRvalue[1] && IRvalue[2] && IRvalue[3]) || (!IRvalue[0] && !IRvalue[1] && IRvalue[2] && IRvalue[3])) {
      Serial.println("Move FORWARD");
      path += 'S';
      front();
    }

    //U Turn Condition [000100] ("U")
    else if ((!IRvalue[1]) && (!IRvalue[2]) && (IRvalue[3]) && (!IRvalue[5])) {
      Serial.println("U-TURN");
      path += 'U';
      uturn();
    }

    //Left Turn Condition [010100] ("L")
    else if ((IRvalue[1] && IRvalue[3]) || (IRvalue[0] && IRvalue[2])) {
      Serial.println("Turn LEFT");
      path += 'L';
      left();
    }

    //Right Turn Condition [000101] ("R")
    if (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && IRvalue[3] && !IRvalue[4] && IRvalue[5]) {
      Serial.println("Turn RIGHT");
      path += 'R';
      right();
    }

    //Stop Condition [111111] Final of the maze
    if (IRvalue[0] && IRvalue[1] && IRvalue[2] && IRvalue[3] && IRvalue[4] && IRvalue[5]) {
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
  //
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
  Serial.println("Front");
  Serial.println(!((IRvalue[3]) && (IRvalue[1] || IRvalue[5])));
  while (!((IRvalue[3]) && (IRvalue[1] || IRvalue[5]))) {
    //
    Serial.print("while");
    Serial.println(!((IRvalue[3]) && (IRvalue[1] || IRvalue[5])));
    motori.run(FORWARD);
    motord.run(FORWARD);
    Serial.println("Avanzar");
    delay(100);

    lineValue(pinIRd, IRvalue, SensorCountTotal);

    // [000001] turn slowly to the right until [000100]
    if ((!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && IRvalue[5]) || (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && IRvalue[4])) {
      gbontrackR();  // correct to the right
    }
    // [010000] turn slowly to the left until [000100]
    else if ((IRvalue[1] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5]) || (IRvalue[0] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5])) {
      gbontrackL();  //correct to the left
    }
    //Go a litle Back if [000000] to try to get back on track
    else if (!IRvalue[0] && !IRvalue[1] && !IRvalue[2] && !IRvalue[3] && !IRvalue[4] && !IRvalue[5]) {
      // Serial.println("Go BACK");
      back();
    }
    lineValue(pinIRd, IRvalue, SensorCountTotal);
  }
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
}

void left() {  //All bifurcations will be at 90° - Using the gyroscope
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
  angle = 0;            // Reset the accumulated angle
  targetAngle = 90;     // Set the target angle
  lastTime = millis();  // Reset the time

  while (angle < targetAngle) {  // Turn until the target angle is reached
    motori.run(BACKWARD);
    motord.run(FORWARD);
    delay(5);
    Serial.println(angle);

    updateGyro();
  }
  delay(100);

  motori.run(FORWARD);
  motord.run(FORWARD);
  delay(10);

  // stop the motors
  stop();
}

void right() {  // -90° Using the gyroscope
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
  angle = 0;            // Reset the accumulated angle
  targetAngle = -90;    // Set the target angle
  lastTime = millis();  // Reset the time

  while (angle < targetAngle) {  // Turn until the target angle is reached
    motori.run(FORWARD);
    motord.run(BACKWARD);
    delay(5);
    Serial.println(angle);

    updateGyro();
  }
  delay(100);

  motori.run(FORWARD);
  motord.run(FORWARD);
  delay(10);

  // stop the motors
  stop();
}

void uturn() {  // With the gyroscope measure a 180-degree turn
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(10);
  angle = 0;            // Reset the accumulated angle
  targetAngle = 180;    // Set the target angle
  lastTime = millis();  // Reset the time

  while (angle < targetAngle) {  // Turn until the target angle is reached
    motori.run(BACKWARD);
    motord.run(FORWARD);
    delay(5);

    updateGyro();
  }
  delay(100);

  motori.run(FORWARD);
  motord.run(FORWARD);
  delay(10);

  // stop the motors
  stop();
}

void updateGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

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
  Serial.println("Ir a la derecha");
  while (!IRvalue[3]) {
    motori.setSpeed(150);
    motord.setSpeed(80);
    delay(10);
    motori.run(FORWARD);
    motord.run(FORWARD);
    delay(10);
    lineValue(pinIRd, IRvalue, SensorCountTotal);
  }
  motori.setSpeed(100);
  motord.setSpeed(100);
  delay(10);
}

void gbontrackL() {  //while is not [001100] do this
  lineValue(pinIRd, IRvalue, SensorCountTotal);
  Serial.println("Ir a la izquierda");
  while (!IRvalue[3]) {
    motori.setSpeed(80);
    motord.setSpeed(150);
    delay(20);
    motori.run(FORWARD);
    motord.run(FORWARD);
    delay(10);
    lineValue(pinIRd, IRvalue, SensorCountTotal);
  }
  motori.setSpeed(100);
  motord.setSpeed(100);
  delay(10);
}

void go(String path) {  // Follow the recorded path
  for (int i = 0; i < path.length(); i++) {
    char step = path[i];
    if (step == 'S') {
      front();
    } else if (step == 'L') left();
    else if (step == 'R') right();
    else if (step == 'U') uturn();
  }
  //uturn();
  stop();
}

void lineValue(const int *pins, int *values, uint8_t count) {
  for (int i = 0; i < count; i++) {
    values[i] = digitalRead(pins[i]);
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
}
