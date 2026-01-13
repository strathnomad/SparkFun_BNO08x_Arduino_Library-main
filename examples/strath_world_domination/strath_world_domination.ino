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
#include <string.h>
#include <math.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

// Workaround: Define sensor report IDs if not found in library header
#ifndef SENSOR_REPORTID_PRESSURE
#define SENSOR_REPORTID_PRESSURE 0x0a
#endif
#ifndef SENSOR_REPORTID_HUMIDITY
#define SENSOR_REPORTID_HUMIDITY 0x0c
#endif
#ifndef SENSOR_REPORTID_TEMPERATURE
#define SENSOR_REPORTID_TEMPERATURE 0x0e
#endif

BNO08x myIMU;

// Control output frequency: true = print every loop (fast), false = print once per second
bool fast_data = false;

// Formatted output: true = print single formatted line with all data every 3 seconds
bool formatted_output = true;

// Console output: true = enable Serial prints, false = disable (UART only mode)
bool enable_console = true;

// UART pins (PC6 = TX, PC7 = RX)
#define UART_TX_PIN PC6
#define UART_RX_PIN PC7
HardwareSerial UartSerial(UART_RX_PIN, UART_TX_PIN);

// UART command buffer
String uartCommand = "";
const int UART_BUFFER_SIZE = 32;

// Threshold warning levels
#define PRESSURE_THRESHOLD_LOW 950.0   // hPa - warning if below this
#define PRESSURE_THRESHOLD_HIGH 1050.0 // hPa - warning if above this
#define HUMIDITY_THRESHOLD_LOW 20.0    // % - warning if below this
#define HUMIDITY_THRESHOLD_HIGH 80.0   // % - warning if above this
#define TEMP_THRESHOLD_LOW 0.0         // °C - warning if below this
#define TEMP_THRESHOLD_HIGH 50.0       // °C - warning if above this

// Threshold warning flags (1 = threshold exceeded, 0 = OK)
bool pressureWarning = 0;
bool humidityWarning = 0;
bool temp1Warning = 0;
bool temp2Warning = 0;
bool temp3Warning = 0;
bool temp4Warning = 0;

// Timing variables for slow output mode
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // 1 second in milliseconds

// Timing variables for formatted output mode
unsigned long lastFormattedPrintTime = 0;
const unsigned long FORMATTED_PRINT_INTERVAL = 3000; // 3 seconds in milliseconds

// Variables to store most recent sensor data for formatted output
struct SensorData {
  bool hasRotationVector;
  float quatI, quatJ, quatK, quatReal, quatRadianAccuracy;
  
  bool hasMagnetometer;
  float magX, magY, magZ;
  byte magAccuracy;
  
  bool hasPressure;
  float pressure;
  
  bool hasHumidity;
  float humidity;
  
  bool hasTemperature;
  float temperature;  // BNO08x temperature (t5)
  
  bool hasThermistors;
  float t1, t2, t3, t4;  // Thermistor temperatures (PA4, PA5, PA6, PA7)
  
  bool hasHeading;
  float headingDegrees;
} latestData;




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

// Thermistor pins (EPCOS/TDK B57871L0103F001 - 10k NTC thermistors with 100k pull-up to 3.3V)
#define THERMISTOR_PIN_1 PA4
#define THERMISTOR_PIN_2 PA5
#define THERMISTOR_PIN_3 PA6
#define THERMISTOR_PIN_4 PA7

// NTC Thermistor parameters (EPCOS/TDK B57871L0103F001 - 10k NTC, B25/100 = 3988K)
#define THERMISTOR_NOMINAL_RESISTANCE 10000  // 10k ohm at 25°C
#define THERMISTOR_NOMINAL_TEMP 25.0         // Temperature for nominal resistance (25°C)
#define THERMISTOR_BETA 3988                // Beta coefficient (B25/100 = 3988 K for B57871L0103F001)
#define PULLDOWN_RESISTANCE 10000            // 10k pull-up resistor to ground
#define ADC_RESOLUTION 1024                
#define VREF 3                         // Reference voltage (3.3V)


//DEFINE MUX PINS
#define ADG708_A0   PA10
#define ADG708_A1   PC9
#define ADG708_A2   PC8
#define ADG708_EN   PA8


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
  // if (enable_console) {
  // Serial.begin(115200);
  //   while(!Serial) delay(10); // Wait for Serial to become available.
  // }
  
  Serial.begin(115200);
  // Initialize UART on PC6 (TX) and PC7 (RX)
  UartSerial.begin(115200);

  pinMode(PB5, OUTPUT); // PS1
  pinMode(PD2, OUTPUT); // PS0
  digitalWrite(PB5, LOW);    // LOW PS1
  digitalWrite(PD2, LOW);    // LOW PS0

  // Configure INT pin as input with pull-up (important!)
  pinMode(BNO08X_INT, INPUT_PULLDOWN);

  pinMode(LED_G, OUTPUT); 
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_G, LOW);       // turn the LED off by making the voltage LOW
  digitalWrite(LED_R, LOW);    // turn the LED off by making the voltage LOW

  // Configure thermistor pins as analog inputs
  pinMode(THERMISTOR_PIN_1, INPUT_ANALOG);
  pinMode(THERMISTOR_PIN_2, INPUT_ANALOG);
  pinMode(THERMISTOR_PIN_3, INPUT_ANALOG);
  pinMode(THERMISTOR_PIN_4, INPUT_ANALOG);

  pinMode(ADG708_A0, OUTPUT);
  pinMode(ADG708_A1, OUTPUT);
  pinMode(ADG708_A2, OUTPUT);
  pinMode(ADG708_EN, OUTPUT);
  digitalWrite(ADG708_EN, HIGH); 

  if (enable_console) {
  Serial.println();
  Serial.println("BNO08x Read Example");
  Serial.println("Initializing I2C bus...");
  }

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

  // Set mounting orientation: X pointing up (vertical), Y right, Z forward/back
  // This is a 90-degree rotation around Y axis to make X vertical instead of Z
  // Quaternion for 90° rotation around Y: i=0, j=sin(45°)=0.707, k=0, real=cos(45°)=0.707
  sh2_Quaternion_t orientation;
  orientation.x = 0.0;
  orientation.y = 0.7071067811865476;
  orientation.z = 0.0;
  orientation.w = 0.7071067811865476;
  if (sh2_setReorientation(&orientation) == SH2_OK) {
    Serial.println(F("Sensor reorientation set for wall mounting (X vertical)"));
  } else {
    Serial.println("Warning: Could not set sensor reorientation");
  }
  delay(100);

  // Initialize sensor data structure
  memset(&latestData, 0, sizeof(latestData));

  Serial.println("Reading events");
  delay(100);

// todo fucked
  if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG) == true) { // all three sensors
  //if (myIMU.setCalibrationConfig(SH2_CAL_ACCEL || SH2_CAL_MAG) == true) { // Default settings
  // if (myIMU.setCalibrationConfig(SH2_CAL_MAG) == true) { // only accel
    Serial.println(F("Calibration Command Sent Successfully"));
  } else {
    Serial.println("Could not send Calibration Command. Freezing...");
    while(1) delay(10);
  }




  
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

  // Enable pressure sensor (using enableReport directly as workaround)
  if (myIMU.enableReport(SENSOR_REPORTID_PRESSURE, 10000) == true) {
    Serial.println(F("Pressure enabled"));
    Serial.println(F("Output in form pressure, in hectopascals"));
  } else {
    Serial.println("Could not enable pressure");
  }

  // Enable humidity sensor (using enableReport directly as workaround)
  if (myIMU.enableReport(SENSOR_REPORTID_HUMIDITY, 10000) == true) {
    Serial.println(F("Humidity enabled"));
    Serial.println(F("Output in form humidity, in percent"));
  } else {
    Serial.println("Could not enable humidity");
  }

  // Enable temperature sensor (using enableReport directly as workaround)
  if (myIMU.enableReport(SENSOR_REPORTID_TEMPERATURE, 10000) == true) {
    Serial.println(F("Temperature enabled"));
    Serial.println(F("Output in form temperature, in Celsius"));
  } else {
    Serial.println("Could not enable temperature");
  }




  // Enable geomagnetic rotation vector for heading (uses magnetometer data)
  if (myIMU.enableGeomagneticRotationVector() == true) {
    Serial.println(F("Geomagnetic Rotation Vector enabled"));
    Serial.println(F("Heading will be calculated from magnetometer data"));
  } else {
    Serial.println("Could not enable geomagnetic rotation vector");
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

  // Check if we should print data based on fast_data setting
  bool shouldPrint = false; //TODO ENABLE PRINTS IF NEEDED FOR DEBUGGING
  if (fast_data) {
    // Fast mode: print every time we get data
    shouldPrint = true;
  } else {
    // Slow mode: print once per second
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
      shouldPrint = true;
      lastPrintTime = currentTime;
    }
  }


  // Has a new event come in on the Sensor Hub Bus?
  // Poll frequently to keep communication alive
  if (myIMU.getSensorEvent() == true) {

    // is it the correct sensor data we want?
    // if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

    //   float quatI = myIMU.getQuatI();
    //   float quatJ = myIMU.getQuatJ();
    //   float quatK = myIMU.getQuatK();
    //   float quatReal = myIMU.getQuatReal();
    //   float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    //   // Store latest data for formatted output and UART commands
    //   latestData.hasRotationVector = true;
    //   latestData.quatI = quatI;
    //   latestData.quatJ = quatJ;
    //   latestData.quatK = quatK;
    //   latestData.quatReal = quatReal;
    //   latestData.quatRadianAccuracy = quatRadianAccuracy;

    //   // Get yaw/heading in radians and convert to degrees
    //   float headingDegrees = (myIMU.getYaw()) * 180.0 / PI;
     
    //   // Normalize heading to 0-360 degrees
    //   if (headingDegrees < 0) {
    //     headingDegrees += 360.0;
    //   }

    //   // Store latest data for formatted output and UART commands
    //   latestData.hasHeading = true;
    //   latestData.headingDegrees = headingDegrees;




    //   if (shouldPrint && !formatted_output) {
    //   Serial.print(quatI, 2);
    //   Serial.print(F(","));
    //   Serial.print(quatJ, 2);
    //   Serial.print(F(","));
    //   Serial.print(quatK, 2);
    //   Serial.print(F(","));
    //   Serial.print(quatReal, 2);
    //   Serial.print(F(","));
    //   Serial.print(quatRadianAccuracy, 2);
    //   Serial.println();
    //   }
    // }

        // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {

      float x = myIMU.getMagX();
      float y = myIMU.getMagY();
      float z = myIMU.getMagZ();
      byte accuracy = myIMU.getMagAccuracy();

      // Store latest data for formatted output and UART commands
      latestData.hasMagnetometer = true;
      latestData.magX = x;
      latestData.magY = y;
      latestData.magZ = z;
      latestData.magAccuracy = accuracy;

      printAccuracyLevel(accuracy);


      if (shouldPrint && !formatted_output) {
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

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_PRESSURE) {

      // Access pressure value directly from sensorValue (workaround)
      float pressure = myIMU.sensorValue.un.pressure.value;

      // Check pressure threshold
      if (pressure < PRESSURE_THRESHOLD_LOW || pressure > PRESSURE_THRESHOLD_HIGH) {
        pressureWarning = 1;
      } else {
        pressureWarning = 0;
      }

      // Store latest data for formatted output and UART commands
      latestData.hasPressure = true;
      latestData.pressure = pressure;

      if (shouldPrint && !formatted_output) {
        Serial.print(F("Pressure: "));
        Serial.print(pressure, 2);
        Serial.print(F(" hPa"));
        if (pressureWarning) {
          Serial.print(F(" [WARNING]"));
        }
        Serial.println();
      }
    }

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_HUMIDITY) {

      // Access humidity value directly from sensorValue (workaround)
      float humidity = myIMU.sensorValue.un.humidity.value;

      // Check humidity threshold
      if (humidity < HUMIDITY_THRESHOLD_LOW || humidity > HUMIDITY_THRESHOLD_HIGH) {
        humidityWarning = 1;
      } else {
        humidityWarning = 0;
      }

      // Store latest data for formatted output and UART commands
      latestData.hasHumidity = true;
      latestData.humidity = humidity;

      if (shouldPrint && !formatted_output) {
        Serial.print(F("Humidity: "));
        Serial.print(humidity, 2);
        Serial.print(F(" %"));
        if (humidityWarning) {
          Serial.print(F(" [WARNING]"));
        }
        Serial.println();
      }
    }

    // is it the correct sensor data we want?
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_TEMPERATURE) {

      // Access temperature value directly from sensorValue (workaround)
      float temperature = myIMU.sensorValue.un.temperature.value;

      // Store latest data for formatted output and UART commands
      latestData.hasTemperature = true;
      latestData.temperature = temperature;

      if (shouldPrint && !formatted_output) {
        Serial.print(F("Temperature: "));
        Serial.print(temperature, 2);
        Serial.println(F(" C"));
      }
    }




    // Calculate heading from geomagnetic rotation vector (uses magnetometer data)
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {

      // Get quaternion values for rotation data
      float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      
      // Store rotation vector data for ROT command
      latestData.hasRotationVector = true;
      latestData.quatI = quatI;
      latestData.quatJ = quatJ;
      latestData.quatK = quatK;
      latestData.quatReal = quatReal;
      latestData.quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

      // Get yaw/heading in radians and convert to degrees
      // With reorientation set, getYaw() now correctly returns heading when rotating around the vertical axis
      float headingDegrees = (myIMU.getYaw()) * 180.0 / PI;

      // Normalize heading to 0-360 degrees
      if (headingDegrees < 0) {
        headingDegrees += 360.0;
      }
      
      // Convert from counter-clockwise to clockwise (standard compass convention)
      // 0° = North, 90° = East, 180° = South, 270° = West
      headingDegrees = 360.0 - headingDegrees;
      if (headingDegrees >= 360.0) {
        headingDegrees = 0.0;
      }
      headingDegrees -= 90.0; // for aic i want the back of the device to face north for now

      // Store latest data for formatted output and UART commands
      latestData.hasHeading = true;
      latestData.headingDegrees = headingDegrees;

      if (shouldPrint && !formatted_output) {
        Serial.print(F("Heading: "));
        Serial.print(headingDegrees, 1);
        Serial.println(F(" degrees"));
      }
    }

  }

  // Read thermistors every loop (for UART commands and formatted output)
  latestData.hasThermistors = true;
  float t1 = readThermistor(THERMISTOR_PIN_1);
  float t2 = readThermistor(THERMISTOR_PIN_2);
  float t3 = readThermistor(THERMISTOR_PIN_3);
  float t4 = readThermistor(THERMISTOR_PIN_4);
  
  latestData.t1 = t1;
  latestData.t2 = t2;
  latestData.t3 = t3;
  latestData.t4 = t4;
  
  // Check temperature thresholds (only if valid reading, not -999.0)
  if (t1 > -900.0) {
    if (t1 < TEMP_THRESHOLD_LOW || t1 > TEMP_THRESHOLD_HIGH) {
      temp1Warning = 1;
    } else {
      temp1Warning = 0;
    }
  }
  
  if (t2 > -900.0) {
    if (t2 < TEMP_THRESHOLD_LOW || t2 > TEMP_THRESHOLD_HIGH) {
      temp2Warning = 1;
    } else {
      temp2Warning = 0;
    }
  }
  
  if (t3 > -900.0) {
    if (t3 < TEMP_THRESHOLD_LOW || t3 > TEMP_THRESHOLD_HIGH) {
      temp3Warning = 1;
    } else {
      temp3Warning = 0;
    }
  }
  
  if (t4 > -900.0) {
    if (t4 < TEMP_THRESHOLD_LOW || t4 > TEMP_THRESHOLD_HIGH) {
      temp4Warning = 1;
    } else {
      temp4Warning = 0;
    }
  }

  // Handle UART commands
  handleUARTCommand();

  // Print formatted output every 3 seconds if enabled
  if (formatted_output) {
    unsigned long currentTime = millis();
    if (currentTime - lastFormattedPrintTime >= FORMATTED_PRINT_INTERVAL) {
      if (enable_console) {
        printFormattedOutput();
      }
      lastFormattedPrintTime = currentTime;
    }
  }
}

// Read NTC thermistor and convert to temperature in Celsius
float readThermistor(int pin) {
  // Read analog value
  int adcValue = analogRead(pin);
  
  // Avoid invalid ADC readings
  if (adcValue <= 0 || adcValue >= ADC_RESOLUTION) {
    return -999.0; // Error value
  }
  

  float voltage = (adcValue / (float)ADC_RESOLUTION) * VREF;
  
  // Avoid division by zero or invalid voltages
  if (voltage <= 0.001 || voltage >= (VREF - 0.001)) {
    return -999.0; // Error value
  }
  
  // Calculate thermistor resistance using voltage divider formula
  float resistance = (PULLDOWN_RESISTANCE/(voltage/VREF)) - PULLDOWN_RESISTANCE ;



  // Avoid invalid resistance values
  if (resistance <= 0.0 || resistance > 1000000.0) {
    return -999.0; // Error value (resistance too high/low)
  }
  
  // Convert resistance to temperature using Beta equation
  // Beta equation: T = 1 / (1/T0 + (1/B) * ln(R/R0))
  // Where T0 = 298.15K (25°C), B = 3988K, R0 = 10k ohm
  // Rearranged: T = 1 / (1/T0 + (ln(R/R0)) / B)
  float T0_Kelvin = THERMISTOR_NOMINAL_TEMP + 273.15;  // 298.15K
  float ratio = resistance / THERMISTOR_NOMINAL_RESISTANCE;  // R/R0
  // Serial.print(resistance);
  // Serial.println(",");
  float logRatio = log(ratio);                               // ln(R/R0)
  float betaTerm = logRatio / THERMISTOR_BETA;               // (1/B) * ln(R/R0)
  float invTemp = (1.0 / T0_Kelvin) + betaTerm;              // 1/T0 + (1/B) * ln(R/R0)
  float tempKelvin = 1.0 / invTemp;                          // T in Kelvin
  float tempCelsius = tempKelvin - 273.15;                   // Convert to Celsius
  
  return tempCelsius;
}



//Given a accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}

// Print formatted output with all available sensor data in a single line
void printFormattedOutput()
{
  Serial.print(F("DATA: "));
  
  // Rotation Vector
  if (latestData.hasRotationVector) {
    Serial.print(F("Quat["));
    Serial.print(latestData.quatI, 2);
    Serial.print(F(","));
    Serial.print(latestData.quatJ, 2);
    Serial.print(F(","));
    Serial.print(latestData.quatK, 2);
    Serial.print(F(","));
    Serial.print(latestData.quatReal, 2);
    Serial.print(F("] "));
  }
  
  // Magnetometer
  if (latestData.hasMagnetometer) {
    Serial.print(F("Mag["));
    Serial.print(latestData.magX, 2);
    Serial.print(F(","));
    Serial.print(latestData.magY, 2);
    Serial.print(F(","));
    Serial.print(latestData.magZ, 2);
    Serial.print(F(","));
    Serial.print(latestData.magAccuracy, 2);
    Serial.print(F("]uT "));
  }
  
  // Pressure
  if (latestData.hasPressure) {
    Serial.print(F("P:"));
    Serial.print(latestData.pressure, 2);
    Serial.print(F("hPa "));
  }
  
  // Humidity
  if (latestData.hasHumidity) {
    Serial.print(F("H:"));
    Serial.print(latestData.humidity, 2);
    Serial.print(F("% "));
  }
  
  // Thermistors (t1-t4)
  if (latestData.hasThermistors) {
    Serial.print(F("t1:"));
    Serial.print(latestData.t1, 2);
    Serial.print(F("C"));
    if (temp1Warning) Serial.print(F("[!]"));
    Serial.print(F(" t2:"));
    Serial.print(latestData.t2, 2);
    Serial.print(F("C"));
    if (temp2Warning) Serial.print(F("[!]"));
    Serial.print(F(" t3:"));
    Serial.print(latestData.t3, 2);
    Serial.print(F("C"));
    if (temp3Warning) Serial.print(F("[!]"));
    Serial.print(F(" t4:"));
    Serial.print(latestData.t4, 2);
    Serial.print(F("C"));
    if (temp4Warning) Serial.print(F("[!]"));
    Serial.print(F(" "));
  }
  
  // BNO08x Temperature (t5)
  if (latestData.hasTemperature) {
    Serial.print(F("t5:"));
    Serial.print(latestData.temperature, 2);
    Serial.print(F("C "));
  }
  
  // Pressure warning flag
  if (latestData.hasPressure) {
    if (pressureWarning) Serial.print(F("P[!] "));
  }
  
  // Humidity warning flag
  if (latestData.hasHumidity) {
    if (humidityWarning) Serial.print(F("H[!] "));
  }
  
  // Heading
  if (latestData.hasHeading) {
    Serial.print(F("Heading:"));
    Serial.print(latestData.headingDegrees, 1);
    Serial.print(F("deg"));
  }
  
  if (enable_console) {
    Serial.println();
  }
}

// ============================================================================
// UART COMMAND HANDLING FUNCTIONS
// ============================================================================

// Send rotation XYZ in degrees (convert from quaternion)
void sendROT() {
  if (latestData.hasRotationVector) {
    float roll = (myIMU.getRoll()) * 180.0 / PI;
    float pitch = (myIMU.getPitch()) * 180.0 / PI;
    float yaw = (myIMU.getYaw()) * 180.0 / PI;
    UartSerial.print(F("["));
    UartSerial.print(roll, 2);
    UartSerial.print(F(","));
    UartSerial.print(pitch, 2);
    UartSerial.print(F(","));
    UartSerial.print(yaw, 2);
    UartSerial.println(F("]"));
  } else {
    UartSerial.println(F("[0,0,0]"));
  }
}

// Send magnetometer XYZ in uT
void sendMAG() {
  if (latestData.hasMagnetometer) {
    UartSerial.print(F("["));
    UartSerial.print(latestData.magX, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.magY, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.magZ, 2);
    UartSerial.println(F("]"));
  } else {
    UartSerial.println(F("[0,0,0]"));
  }
}

// Send pressure in hPa
void sendBAR() {
  if (latestData.hasPressure) {
    UartSerial.println(latestData.pressure, 2);
  } else {
    UartSerial.println(F("0"));
  }
}

// Send humidity in %
void sendHUM() {
  if (latestData.hasHumidity) {
    UartSerial.println(latestData.humidity, 2);
  } else {
    UartSerial.println(F("0"));
  }
}

// Send temperatures: [t1, t2, t3, t4, t5] in C
void sendTEMP() {
  UartSerial.print(F("["));
  if (latestData.hasThermistors) {
    UartSerial.print(latestData.t1, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.t2, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.t3, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.t4, 2);
  } else {
    UartSerial.print(F("0,0,0,0"));
  }
  UartSerial.print(F(","));
  if (latestData.hasTemperature) {
    UartSerial.print(latestData.temperature, 2);
  } else {
    UartSerial.print(F("0"));
  }
  UartSerial.println(F("]"));
}

// Send heading in degrees
void sendHEAD() {
  if (latestData.hasHeading) {
    UartSerial.println(latestData.headingDegrees, 1);
  } else {
    UartSerial.println(F("0"));
  }
}

// Send all data: [ROT, MAG, BAR, HUM, TEMP, HEAD]
void sendDATA() {
  UartSerial.print(F("["));
  
  // ROT
  if (latestData.hasRotationVector) {
    float roll = (myIMU.getRoll()) * 180.0 / PI;
    float pitch = (myIMU.getPitch()) * 180.0 / PI;
    float yaw = (myIMU.getYaw()) * 180.0 / PI;
    UartSerial.print(F("["));
    UartSerial.print(roll, 2);
    UartSerial.print(F(","));
    UartSerial.print(pitch, 2);
    UartSerial.print(F(","));
    UartSerial.print(yaw, 2);
    UartSerial.print(F("]"));
  } else {
    UartSerial.print(F("[0,0,0]"));
  }
  UartSerial.print(F(","));
  
  // MAG
  if (latestData.hasMagnetometer) {
    UartSerial.print(F("["));
    UartSerial.print(latestData.magX, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.magY, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.magZ, 2);
    UartSerial.print(F("]"));
  } else {
    UartSerial.print(F("[0,0,0]"));
  }
  UartSerial.print(F(","));
  
  // BAR
  if (latestData.hasPressure) {
    UartSerial.print(latestData.pressure, 2);
  } else {
    UartSerial.print(F("0"));
  }
  UartSerial.print(F(","));
  
  // HUM
  if (latestData.hasHumidity) {
    UartSerial.print(latestData.humidity, 2);
  } else {
    UartSerial.print(F("0"));
  }
  UartSerial.print(F(","));
  
  // TEMP
  UartSerial.print(F("["));
  if (latestData.hasThermistors) {
    UartSerial.print(latestData.t1, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.t2, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.t3, 2);
    UartSerial.print(F(","));
    UartSerial.print(latestData.t4, 2);
  } else {
    UartSerial.print(F("0,0,0,0"));
  }
  UartSerial.print(F(","));
  if (latestData.hasTemperature) {
    UartSerial.print(latestData.temperature, 2);
  } else {
    UartSerial.print(F("0"));
  }
  UartSerial.print(F("]"));
  UartSerial.print(F(","));
  
  // HEAD
  if (latestData.hasHeading) {
    UartSerial.print(latestData.headingDegrees, 1);
  } else {
    UartSerial.print(F("0"));
  }
  
  UartSerial.println(F("]"));
}

// Send flags: [BAR FLAG, HUM FLAG, T1 FLAG, T2 FLAG, T3 FLAG, T4 FLAG]
void sendFLAG() {
  UartSerial.print(F("["));
  UartSerial.print(pressureWarning ? 1 : 0);
  UartSerial.print(F(","));
  UartSerial.print(humidityWarning ? 1 : 0);
  UartSerial.print(F(","));
  UartSerial.print(temp1Warning ? 1 : 0);
  UartSerial.print(F(","));
  UartSerial.print(temp2Warning ? 1 : 0);
  UartSerial.print(F(","));
  UartSerial.print(temp3Warning ? 1 : 0);
  UartSerial.print(F(","));
  UartSerial.print(temp4Warning ? 1 : 0);
  UartSerial.println(F("]"));
}

// Parse and handle UART commands
void handleUARTCommand() {
  if (UartSerial.available() > 0) {
    String command = UartSerial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "ROT") {
      sendROT();
    } else if (command == "MAG") {
      sendMAG();
    } else if (command == "BAR") {
      sendBAR();
    } else if (command == "HUM") {
      sendHUM();
    } else if (command == "TEMP") {
      sendTEMP();
    } else if (command == "HEAD") {
      sendHEAD();
    } else if (command == "DATA") {
      sendDATA();
    } else if (command == "FLAG") {
      sendFLAG();
    } 
    else if (command == "s"){
      // Saves the current dynamic calibration data (DCD) to memory
      // Note, The BNO08X stores updated Dynamic Calibration Data (DCD) to RAM 
      // frequently (every 5 seconds), so this command may not be necessary
      // depending on your application.
      if (myIMU.saveCalibration() == true) {
        Serial.println(F("Calibration data was saved successfully"));
      } else {
        Serial.println("Save Calibration Failure");
      }
    }else if (command.startsWith("PD")) {
      
      // Format: PDN: PD1, PD2, PD3, PD4, PD5, PD6
      if (command == "PD3") { // NO2 -> SIG3 -> s1
        digitalWrite(ADG708_A2, LOW);
        digitalWrite(ADG708_A1, LOW);
        digitalWrite(ADG708_A0, LOW);
        UartSerial.print("PD3 OK");
      }
      else if (command == "PD4") { // NO3 -> SIG4 -> s2
      // PD2
        digitalWrite(ADG708_A2, LOW);
        digitalWrite(ADG708_A1, LOW);
        digitalWrite(ADG708_A0, HIGH);
        UartSerial.print("PD4 OK");
        // Serial.println("NO3 detected -> SIG4 -> ADG708 channel s2");
      }
      else if (command == "PD5") { // NO4 -> SIG5 -> s3
      // PD3
        digitalWrite(ADG708_A2, LOW);
        digitalWrite(ADG708_A1, HIGH);
        digitalWrite(ADG708_A0, LOW);
        UartSerial.print("PD5 OK");
        // Serial.println("NO4 detected -> SIG5 -> ADG708 channel s3");
      }
      else if (command == "PD6") { // NO5 -> SIG6 -> s4
      // PD4
        digitalWrite(ADG708_A2, LOW);
        digitalWrite(ADG708_A1, HIGH);
        digitalWrite(ADG708_A0, HIGH);
        UartSerial.print("PD6 OK");
        // Serial.println("NO5 detected -> SIG6 -> ADG708 channel s4");
      }
      else if (command == "PD1") {      // NO0 -> SIG1 -> s5
      // PD5
        digitalWrite(ADG708_A2, HIGH);
        digitalWrite(ADG708_A1, LOW);
        digitalWrite(ADG708_A0, LOW);
        UartSerial.print("PD1 OK");
        // Serial.println("NO0 detected -> SIG1 -> ADG708 channel s5");
      }
      else if (command == "PD2") { // NO1 -> SIG2 -> s6
      // PD6
        digitalWrite(ADG708_A2, HIGH);
        digitalWrite(ADG708_A1, LOW);
        digitalWrite(ADG708_A0, HIGH);
        UartSerial.print("PD2 OK");
        // Serial.println("NO1 detected -> SIG2 -> ADG708 channel s6");
      }

    }
  }
}

