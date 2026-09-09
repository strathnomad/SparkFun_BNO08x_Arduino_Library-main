/*
  Using the BNO08x IMU

  Stripped-down version of strath_world_domination that only reads and
  prints acceleration data (x, y, z, accuracy) at a fixed interval.

  Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

BNO08x myIMU;

// Create custom TwoWire instance with specific pins (like the working Adafruit code)
TwoWire myWire(PB3, PB10);  // SDA=PB3, SCL=PB10

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
#define BNO08X_INT  PC10
#define BNO08X_RST  PC11

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

// Print interval
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // milliseconds between prints

void setup() {
  Serial.begin(115200);

  pinMode(PB5, OUTPUT); // PS1
  pinMode(PD2, OUTPUT); // PS0
  digitalWrite(PB5, LOW);    // LOW PS1
  digitalWrite(PD2, LOW);    // LOW PS0

  pinMode(BNO08X_INT, INPUT_PULLDOWN);

  Serial.println();
  Serial.println("BNO08x Acceleration Example");
  Serial.println("Initializing I2C bus...");

  myWire.begin();
  myWire.setClock(400000);
  delay(100);

  pinMode(BNO08X_RST, OUTPUT);
  digitalWrite(BNO08X_RST, LOW);
  delay(10);
  digitalWrite(BNO08X_RST, HIGH);
  delay(1500); // Wait for BNO08x to boot after reset

  Serial.print("Attempting to connect to BNO08x... ");
  if (myIMU.begin(BNO08X_ADDR, myWire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("Failed to connect. Freezing...");
    while (1) {
      delay(1000);
      Serial.print(".");
    }
  }
  Serial.println("Connected!");

  // Wait for INT pin to go low (indicates sensor is ready for communication)
  Serial.println("Waiting for INT pin to go low (sensor ready)...");
  int intWaitCount = 0;
  while (digitalRead(BNO08X_INT) == HIGH && intWaitCount < 100) {
    delay(10);
    intWaitCount++;
  }
  if (intWaitCount >= 100) {
    Serial.println("Warning: INT pin did not go low, continuing anyway...");
  } else {
    Serial.println("INT pin is low - sensor ready!");
  }
  delay(100); // Additional delay for stability

  setReports();
  delay(200); // Give sensor time to process report enable

  Serial.println("Reading events");
}

void setReports(void) {
  Serial.println("Setting desired reports");

  // Wait for INT pin to be ready before enabling reports
  int retryCount = 0;
  while (digitalRead(BNO08X_INT) == HIGH && retryCount < 50) {
    delay(10);
    retryCount++;
  }

  if (myIMU.enableAccelerometer() == true) {
    Serial.println(F("Acceleration vector enabled"));
    Serial.println(F("Output in form x, y, z, in m/s^2"));
    delay(100); // Give sensor time to process
  } else {
    Serial.println("Could not enable acceleration vector");
    Serial.println("Retrying in 200ms...");
    delay(200);
    // Retry once
    if (myIMU.enableAccelerometer() == true) {
      Serial.println(F("Acceleration vector enabled on retry"));
      delay(100);
    } else {
      Serial.println("Failed to enable acceleration vector after retry");
    }
  }
}

void loop() {
  delay(5);

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    delay(100); // Wait after reset
    setReports();
    delay(100); // Additional delay after setting reports
  }

  if (myIMU.getSensorEvent() == true) {

    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {

      float x = myIMU.getAccelX();
      float y = myIMU.getAccelY();
      float z = myIMU.getAccelZ();
      byte accuracy = myIMU.getAccelAccuracy();

      unsigned long currentTime = millis();
      if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print(x, 4);
        Serial.print(F(","));
        Serial.print(y, 4);
        Serial.print(F(","));
        Serial.print(z, 4);
        Serial.println();

        lastPrintTime = currentTime;
      }
    }
  }
}
