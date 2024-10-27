/*
  Arduino Analog Inputs Reader
  Reads analog inputs A0 to A5 at 100 Hz and sends data over Serial using sprintf.
*/

const int NUM_SENSORS = 7;        // Number of analog sensors
const int analogPins[NUM_SENSORS] = {A0, A1, A2, A4, A5, A6, A7}; // Analog pins A0 to A6
const unsigned long SAMPLE_INTERVAL = 20; // 10 ms interval for 100 Hz

unsigned long previousMillis = 0;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200); // Baud rate should match the Python script
  while (!Serial) {
    ; // Wait for Serial port to connect. Needed for native USB
  }
  Serial.println("Sensor1,Sensor2,Sensor3,Sensor4,Sensor5,Sensor6"); // Header
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to read sensors
  if (currentMillis - previousMillis >= SAMPLE_INTERVAL) {
    previousMillis = currentMillis;

    // Read all sensors
    int sensorValues[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = 0;
    }

    for (int j = 0; j < 64; ++j) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] += analogRead(analogPins[i]);
      }
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] >>=6;
    }
    

    // Format the data into a CSV string
    char buffer[64]; // Buffer to hold the formatted string
    // Ensure that buffer size is sufficient
    // Each sensor value can be up to 4 digits (0-1023), plus commas and newline
    sprintf(buffer, "%03x,%03x,%03x,%03x,%03x,%03x,%03x\n",
            sensorValues[0],
            sensorValues[1],
            sensorValues[2],
            sensorValues[3],
            sensorValues[4],
            sensorValues[5],
            sensorValues[6]);

    // Send the formatted string over Serial
    Serial.print(buffer);
  }

  // Other non-blocking code can go here
}
