// Line Maze Solver (LSRB Algorithm)

//Motors
#include <AFMotor.h>
AF_DCMotor motori(1);  //Left motor - connected to terminal 1
AF_DCMotor motord(2);  //Right motor - connected to terminal 2

// Reflectance Sensors TCRT5000
const uint8_t SensorCount = 6;
const int pinIRd[SensorCount] = { 2, 4, 6, 8, 10, 12 };                    // Pines para los sensores infrarrojos
int IRvalue[SensorCount] = { 0, 0, 0, 0, 0, 0 };  // Valores leídos de los sensores - high (on - black) or low (off - white)


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
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup");

  //Configurando motores
  motori.setSpeed(150);
  motori.run(RELEASE);

  motord.setSpeed(150);
  motord.run(RELEASE);
  delay(100);

  // MPU Configuration
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  lastTime = millis();

  // configure the sensors
  for (int i = 0; i < 6; i++) {
    pinMode(pinIRd[i], INPUT);  // Configura cada pin como entrada
  }


  delay(500);
}

void loop() {
  // Read Values
  lineValue();
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

  for (int i = 0; i < 6; i++) {
   // Lee cada sensor y almacena el valor en su posición correspondiente
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(IRvalue[i]);  // Mostrar valores en el monitor serial
  }
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

void front() {
  //
  motori.run(FORWARD);
  motord.run(FORWARD);
  delay(100);
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

    lineValue();

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
    lineValue();
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
  lineValue();
  Serial.println("Ir a la derecha");
  while (!IRvalue[3]) {
    motori.setSpeed(150);
    motord.setSpeed(80);
    delay(10);
    motori.run(FORWARD);
    motord.run(FORWARD);
    delay(10);
    lineValue();
  }
  motori.setSpeed(100);
  motord.setSpeed(100);
  delay(10);
}

void gbontrackL() {  //while is not [001100] do this
  lineValue();
  Serial.println("Ir a la izquierda");
  while (!IRvalue[3]) {
    motori.setSpeed(80);
    motord.setSpeed(150);
    delay(20);
    motori.run(FORWARD);
    motord.run(FORWARD);
    delay(10);
    lineValue();
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

void lineValue() {
  for (int i = 0; i < 6; i++) {
    IRvalue[i] = digitalRead(pinIRd[i]);  // Lee cada sensor y almacena el valor en su posición correspondiente
    //Serial.print("Sensor ");
    //Serial.print(i);
    //Serial.print(": ");
    //Serial.println(IRvalue[i]);  // Mostrar valores en el monitor serial
  }
  delay(10);
}
