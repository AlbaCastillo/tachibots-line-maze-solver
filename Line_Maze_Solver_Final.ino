// Line Follower with PD Control

#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Motors
AF_DCMotor motori(1);  // Left motor - connected to terminal 1
AF_DCMotor motord(2);  // Right motor - connected to terminal 2

// Speed Constants
const int baseSpeed = 90;  // Base speed for motors
const int MAX_SPEED = 255; // Maximum speed

// PD Control Constants
float Kp = 10.0;  // Start with a lower value
float Kd = 2.0;   // Adjust based on testing
float Ki = 0.2;

// Error variables
int error = 0;
int errorLast = 0;
int derivative = 0;
int errorSquared;
int errorSum = 0; // For Integral term

// Sensor arrays
const uint8_t SensorCountSetup = 2;  // Sensors initialized in setup
const uint8_t SensorCountPost = 4;   // Sensors initialized after motors start
const uint8_t SensorCountTotal = SensorCountSetup + SensorCountPost;  // Total sensors

// Sensor position mapping
const int pinIRdSetup[SensorCountSetup] = {10, 9};          // Pins configured in setup
const int pinIRdPost[SensorCountPost] = {6, 0, 2, 12};      // Pins configured post-start

int pinIRd[SensorCountTotal];  // Combined sensor pins
int IRvalue[SensorCountTotal]; // Sensor readings

bool calibrated = false;           // Tracks whether calibration is complete
bool postSensorsInitialized = false;  // Tracks whether post sensors are initialized

const int MAX_INTEGRAL = 1000;
float correction;
float motorLSpeed;
float motorRSpeed;

// Variable to keep track of the last extreme sensor
int lastExtremeSensor = 0; // -1 for left, 1 for right, 0 if none

// Function prototypes
void setup();
void loop();
void lineValue(const int *pins, int *values, uint8_t count);
void debugInfraRed(const int *values, uint8_t count, const char *label);
void performCalibration();
void initializePostSensors();
int calculateError();
void setMotor(AF_DCMotor &motor, int speed);
void stop();


// Setup function (unchanged)
void setup() {
  Serial.begin(9600);
  Serial.println("Setup");

  // Configuring motors
  motori.setSpeed(baseSpeed);
  motord.setSpeed(baseSpeed);
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(100);

  // Configure pins for setup sensors
  for (int i = 0; i < SensorCountSetup; i++) {
    pinMode(pinIRdSetup[i], INPUT_PULLUP);
  }

  delay(500);
}

// Main loop function
void loop() {
  if (!calibrated) {
    performCalibration();
  } else {
    // Initialize post sensors after motors start
    if (!postSensorsInitialized) {
      initializePostSensors();
    }

    // Read sensor values
    lineValue(pinIRd, IRvalue, SensorCountTotal);

    // Update last extreme sensor detection
    if (IRvalue[0] == LOW) { // Front left sensor detects line
      lastExtremeSensor = -1;
    } else if (IRvalue[4] == LOW) { // Front right sensor detects line
      lastExtremeSensor = 1;
    }

    // PD control
    errorLast = error;
    error = calculateError();
    derivative = error - errorLast;
    errorSquared = error * error * (error < 0 ? -1 : 1); // Preserve the sign

    // Update error sum for integral term
    errorSum += error;
    errorSum = constrain(errorSum, -MAX_INTEGRAL, MAX_INTEGRAL);

    // Limit correction
    int maxCorrection = baseSpeed; // Adjust as necessary

    // correction = (Kp * error) + (Kd * derivative) + (Ki * errorSum);
    correction = (Kp * errorSquared) + (Kd * derivative);
    correction = constrain(correction, -maxCorrection, maxCorrection);

    // Detect extreme deviations
    bool allSensorsHigh = true;
    for (int i = 0; i < SensorCountTotal; i++) {
      if (IRvalue[i] == HIGH) {
        allSensorsHigh = false;
        break;
      }
    }

    motorLSpeed = baseSpeed;
    motorRSpeed = baseSpeed;

     // Handle extreme cases
    if (allSensorsHigh) {
      // Lost line, initiate recovery based on last extreme sensor
      if (lastExtremeSensor == 1) {
        // Last detected on right, turn right
        motord.run(RELEASE);
        motorRSpeed = -baseSpeed;  // Reverse right motor
      } else {
        // Last detected on left, turn left or no extreme sensor detected before losing line, default recovery
        motori.run(RELEASE);
        motorLSpeed = -baseSpeed; // Reverse left motor
      }
    } else {
      // Regular PD control
      if (correction >= 0) {
        // Turn left: slow down left motor
        motorLSpeed = baseSpeed - correction;
      } else if (correction < 0) {
        // Turn right: slow down right motor
        motorRSpeed = baseSpeed + correction; // correction is negative
      }

    }

    // Set motor speeds
    setMotor(motori, motorLSpeed);
    setMotor(motord, motorRSpeed);
    debugInfraRed(IRvalue, SensorCountTotal, "Sensor States"); // Uncomment for debugging
    Serial.print(" | U Turn : "); Serial.print(allSensorsHigh);
    Serial.print(" | Last Extreme: "); Serial.print(lastExtremeSensor);
    Serial.print(" | ErrorSquared: "); Serial.print(Kp * errorSquared);
    Serial.print(" | Derivative: "); Serial.print(Kd * derivative);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | Correction: "); Serial.print(correction);
    Serial.print(" | Left Speed: "); Serial.print(motorLSpeed);
    Serial.print(" | Right Speed: "); Serial.println(motorRSpeed);

    // Small delay for stability
    delay(50);
  }
}


// Function to calculate error based on sensor readings
int calculateError() {
  int sensorWeights[6] = {-12, -6, 0, 0, 6, 12}; // Adjusted weights for symmetry
  int weightedSum = 0;
  int activeSensors = 0;

  for (int i = 0; i < SensorCountTotal; i++) {
    if (IRvalue[i] == LOW) { // Assuming LOW indicates line detected
      weightedSum += sensorWeights[i];
      activeSensors++;
    }
  }

  if (activeSensors > 0) {
    return weightedSum / activeSensors;
  } else {
    // No line detected, continue in last known direction
    if (errorLast < 0) {
      return -6; // Max error to the left
    } else {
      return 6; // Max error to the right
    }
  }
}

void setMotor(AF_DCMotor &motor, int speed) {
  if (speed >= 0) {
    motor.setSpeed(speed);
    motor.run(FORWARD);
  } else {
    motor.setSpeed(-speed); // speed is negative
    motor.run(BACKWARD);
  }
  delay(10);
}

// Function to read sensor values
void lineValue(const int *pins, int *values, uint8_t count) {
  for (int i = 0; i < count; i++) {
    values[i] = digitalRead(pins[i]);
  }
  delay(5);
}

// Function to print sensor debug messages
void debugInfraRed(const int *values, uint8_t count, const char *label) {
  Serial.print(label);
  Serial.print(": ");
  for (int i = 0; i < count; i++) {
    Serial.print(values[i]);
    Serial.print(" ");
  }
  Serial.print("");
}

// Function to perform calibration (unchanged)
void performCalibration() {
  Serial.println("Calibrating...");

  // Check if all setup sensors are in air (all HIGH)
  bool allSensorsHigh = true;
  lineValue(pinIRdSetup, IRvalue, SensorCountSetup);

  for (int i = 0; i < SensorCountSetup; i++) {
    if (IRvalue[i] == LOW) {
      allSensorsHigh = false;
    }
  }

  debugInfraRed(IRvalue, SensorCountSetup, "Setup Sensor States");

  if (allSensorsHigh) {
    Serial.println("All setup sensors detect air. Waiting for black signal...");

    // Wait until any setup sensor detects black (LOW)
    bool blackDetected = false;
    while (!blackDetected) {
      lineValue(pinIRdSetup, IRvalue, SensorCountSetup);
      for (int i = 0; i < SensorCountSetup; i++) {
        if (IRvalue[i] == LOW) {
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
    // motori.run(FORWARD);
    // motord.run(FORWARD);
  }

  delay(100);  // Small delay for stability
}

// Function to initialize post sensors (unchanged)
void initializePostSensors() {
  Serial.println("Initializing post sensors...");

  // Initialize post sensors
  for (int i = 0; i < SensorCountPost; i++) {
    pinMode(pinIRdPost[i], INPUT_PULLUP);
  }

  // Combine sensor pins
  pinIRd[0] = pinIRdPost[0];  // Front Left
  pinIRd[1] = pinIRdPost[1];  // Rear Left
  pinIRd[2] = pinIRdSetup[0]; // Front Center
  pinIRd[3] = pinIRdSetup[1]; // Rear Center
  pinIRd[4] = pinIRdPost[2];  // Front Right
  pinIRd[5] = pinIRdPost[3];  // Rear Right

  postSensorsInitialized = true;
  Serial.println("Post sensors initialized and combined array ready.");

  // Normal operation after calibration and post sensor initialization
  Serial.println("Calibrated and post sensors initialized. Monitoring sensors...");
}

// Function to handle forks with left path priority
void handleForks() {
  Serial.println("Fork detected, turning left");

  // Turn left by slowing down the left motor
  motori.setSpeed(baseSpeed / 2);  // Reduce speed of left motor
  motord.setSpeed(baseSpeed);      // Maintain speed on right motor
  // motori.run(FORWARD);
  // motord.run(FORWARD);

  // Continue until the fork is passed
  // You might need to implement additional logic to determine when to stop turning
}

// Function to stop the robot
void stop() {
  motori.run(RELEASE);
  motord.run(RELEASE);
  delay(50);
}
