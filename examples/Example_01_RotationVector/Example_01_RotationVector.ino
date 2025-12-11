/*
  Using the BNO08x IMU

  This example shows how to output the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  Hardware Connections:
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT
  A5  --> RST

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

// Create custom TwoWire instance with specific pins (like the working Adafruit code)
TwoWire myWire(PB3, PB10);  // SDA=PB3, SCL=PB10

// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.

#define BNO08X_INT  PC10
#define BNO08X_RST  PC11
//#define BNO08X_RST  -1
#define LED_G PB14
#define LED_R PB15


#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
// #define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

// I2C Bus Recovery function - attempts to clear stuck bus
void recoverI2CBus() {
  Serial.println("Attempting I2C bus recovery...");
  pinMode(PB3, OUTPUT); // SDA
  pinMode(PB10, OUTPUT); // SCL
  
  // Generate clock pulses to clear stuck devices
  for(int i = 0; i < 9; i++) {
    digitalWrite(PB10, LOW);
    delayMicroseconds(5);
    digitalWrite(PB10, HIGH);
    delayMicroseconds(5);
  }
  
  // Try to release SDA
  digitalWrite(PB3, HIGH);
  delayMicroseconds(5);
  
  // Reconfigure as inputs (will be set by Wire.begin)
  pinMode(PB3, INPUT);
  pinMode(PB10, INPUT);
  delay(10);
}

// I2C Scanner function with detailed error reporting
void scanI2C(TwoWire &wirePort) {
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  int unknownErrors = 0;

  for(address = 1; address < 127; address++ ) {
    wirePort.beginTransmission(address);
    error = wirePort.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 1) {
      // Data too long - not a real error for scanning
    }
    else if (error == 2) {
      // NACK on transmit of address - normal, no device
    }
    else if (error == 3) {
      // NACK on transmit of data - device exists but issue
      Serial.print("Device at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" - NACK on data");
    }
    else if (error == 4) {
      unknownErrors++;
      if (address <= 10) { // Only print first few to avoid spam
        Serial.print("Unknown error (bus issue) at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  }
  
  if (unknownErrors > 0) {
    Serial.print("\n*** I2C BUS PROBLEM DETECTED ***");
    Serial.print("\nGot 'Unknown error' (code 4) at ");
    Serial.print(unknownErrors);
    Serial.println(" addresses - this indicates I2C bus issue!");
    Serial.println("Possible causes:");
    Serial.println("1. Missing pull-up resistors on SDA/SCL (need 4.7k to 3.3V)");
    Serial.println("2. SDA or SCL lines shorted or stuck");
    Serial.println("3. Wrong pin assignments");
    Serial.println("4. I2C clock speed too high");
    Serial.println("5. Power supply issues");
    Serial.println();
  }
  
  if (nDevices == 0 && unknownErrors == 0) {
    Serial.println("No I2C devices found (normal if nothing connected)\n");
  } else if (nDevices > 0) {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)\n");
  }
}

void setup() {
  Serial.begin(115200);
  
  while(!Serial) delay(10); // Wait for Serial to become available.

  pinMode(PB5, OUTPUT); // PS1
  pinMode(PD2, OUTPUT); // PS0
  digitalWrite(PB5, LOW);    // LOW PS1
  digitalWrite(PD2, LOW);    // LOW PS0

  // Configure INT pin as input with pull-up (important!)
  pinMode(BNO08X_INT, INPUT_PULLUP);

  pinMode(LED_G, OUTPUT); 
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_G, LOW);       // turn the LED off by making the voltage LOW
  digitalWrite(LED_R, HIGH);    // turn the LED off by making the voltage LOW

  Serial.println();
  Serial.println("BNO08x Read Example");
  Serial.println("Initializing I2C bus...");

  // Initialize custom TwoWire instance (like the working Adafruit code)
  myWire.begin();
  // Try 400kHz first, but if you get continuous resets, try 100000 instead
  myWire.setClock(400000); // Set to 400kHz like working code
  // myWire.setClock(100000); // Uncomment this and comment above if you get continuous resets
  delay(100); // Give I2C bus time to stabilize

  // Reset pulse sequence (may help with connection)
  pinMode(BNO08X_RST, OUTPUT);
  digitalWrite(BNO08X_RST, LOW);
  delay(10);
  digitalWrite(BNO08X_RST, HIGH);
  delay(1500); // Wait for BNO08x to boot after reset

  // Scan I2C bus to see what devices are present
  scanI2C(myWire);
  delay(500);

  // Try both addresses like the working Adafruit code does
  Serial.println("Attempting to connect to BNO08x...");
  bool connected = false;
  
  // Try 0x4A first (if ADR jumper is closed) - matches working code order
  Serial.print("Trying address 0x4A... ");
  if (myIMU.begin(0x4A, myWire, BNO08X_INT, BNO08X_RST) == true) {
    Serial.println("SUCCESS!");
    connected = true;
  } else {
    Serial.println("Failed");
    delay(100);
    
    // Try 0x4B (default address)
    Serial.print("Trying address 0x4B... ");
    if (myIMU.begin(0x4B, myWire, BNO08X_INT, BNO08X_RST) == true) {
      Serial.println("SUCCESS!");
      connected = true;
    } else {
      Serial.println("Failed");
    }
  }
  
  if (!connected) {
    Serial.println("\n*** FAILED TO CONNECT ***");
    Serial.println("\nCRITICAL: If you saw 'Unknown error' messages above,");
    Serial.println("this is an I2C BUS HARDWARE problem, not a software issue!");
    Serial.println("\nTroubleshooting steps:");
    Serial.println("1. CHECK PULL-UP RESISTORS:");
    Serial.println("   - SDA (PB3) needs 4.7kΩ pull-up to 3.3V");
    Serial.println("   - SCL (PB10) needs 4.7kΩ pull-up to 3.3V");
    Serial.println("   - Qwiic boards have these built-in, verify connections");
    Serial.println("2. VERIFY WIRING:");
    Serial.println("   - SDA = PB3");
    Serial.println("   - SCL = PB10");
    Serial.println("   - INT = PC10");
    Serial.println("   - RST = PC11");
    Serial.println("   - Power and Ground");
    Serial.println("3. CHECK FOR SHORTS:");
    Serial.println("   - Measure continuity between SDA/SCL and GND/VCC");
    Serial.println("   - Should be open circuit (no continuity)");
    Serial.println("4. POWER SUPPLY:");
    Serial.println("   - BNO08x needs stable 3.3V power");
    Serial.println("   - Check voltage at BNO08x VCC pin");
    Serial.println("5. TRY LOWER I2C SPEED:");
    Serial.println("   - Change Wire.setClock(400000) to Wire.setClock(100000)");
    Serial.println("6. VERIFY PS0/PS1:");
    Serial.println("   - Both should be LOW for I2C mode");
    Serial.println("   - Current: PS0=LOW, PS1=LOW (correct)");
    while(1) {
      delay(1000);
      Serial.print(".");
    }
  }
  
  Serial.println("BNO08x found and connected!");

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
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  
  // Wait for INT pin to be ready before enabling reports
  int retryCount = 0;
  while (digitalRead(BNO08X_INT) == HIGH && retryCount < 50) {
    delay(10);
    retryCount++;
  }
  
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form i, j, k, real, accuracy"));
    delay(100); // Give sensor time to process
  } else {
    Serial.println("Could not enable rotation vector");
    Serial.println("Retrying in 200ms...");
    delay(200);
    // Retry once
    if (myIMU.enableRotationVector() == true) {
      Serial.println(F("Rotation vector enabled on retry"));
      delay(100);
    } else {
      Serial.println("Failed to enable rotation vector after retry");
    }
  }


   if (myIMU.enableMagnetometer() == true) {
    Serial.println(F("Magnetometer enabled"));
    Serial.println(F("Output in form x, y, z, in uTesla"));
  } else {
    Serial.println("Could not enable magnetometer");
  }


}

void loop() {
  delay(5); // Reduced delay for more frequent polling

  if (myIMU.wasReset()) {
    Serial.print("sensor was reset ");
    delay(100); // Wait after reset
    setReports();
    delay(100); // Additional delay after setting reports
  }

  // Has a new event come in on the Sensor Hub Bus?
  // Poll frequently to keep communication alive
  if (myIMU.getSensorEvent() == true) {

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

      float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.print(F(","));
      Serial.print(quatReal, 2);
      Serial.print(F(","));
      Serial.print(quatRadianAccuracy, 2);
      Serial.println();
    }

        // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {

      float x = myIMU.getMagX();
      float y = myIMU.getMagY();
      float z = myIMU.getMagZ();
      byte accuracy = myIMU.getMagAccuracy();

      Serial.print(x, 2);
      Serial.print(F(","));
      Serial.print(y, 2);
      Serial.print(F(","));
      Serial.print(z, 2);
      Serial.print(F(","));
      printAccuracyLevel(accuracy);

      Serial.println();
    }


  }
}



//Given a accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}

