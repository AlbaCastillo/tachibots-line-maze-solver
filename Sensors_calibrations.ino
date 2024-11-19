// TCRT5000 Sensor Test Code
// This code reads analog values from the sensors and prints them to the Serial Monitor
// int THRESHOLD = ((whiteValue + blackValue) / 2) + OFFSET; // OFFSET could be 20, for example


// Number of sensors (adjust if you have more or fewer sensors)
const uint8_t SensorCount = 6;

// Sensor pins (adjust these pins according to your wiring)
const int sensorPins[SensorCount] = {A0, A1, A2, A3, A4, A5};

// Variables to store sensor readings
int sensorValues[SensorCount];

void setup() {
  Serial.begin(9600);
  Serial.println("TCRT5000 Sensor Calibration Test");
  Serial.println("Place the sensors over different surfaces to read values.");
  Serial.println("------------------------------------------------------");
}

void loop() {
  // Read sensor values
  for (int i = 0; i < SensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Print sensor values
  Serial.print("Sensor Values: ");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(500); // Adjust delay as needed
}
