// Sensor control program for Blindsighted. Ver. 1.00, 1/21/2020
// Written in collaboration by Garrisen Cizmich and Jonah Cullen, based on code provided by Peter Jansen.
// Reads distance inputs from each sensor and adjusts motor response accordingly.

/* Pins:
  0 -
  1 -
  2 - Mux LSB
  3 - Mux bit 1
  4 - Mux bit 2
  5 - Mux MSB
  6 - Sensor TX
  7 - Sensor RX
  8 -
  9 - 
  10 - 
  11 -
  12 - 
  13 - 

  A1 -
  A2 -
  A3 -
  A4 - SDA
  A5 - SCL
  
  
*/

#include <SoftwareSerial.h>
#include "TFMini.h"

// Setup software serial port 
SoftwareSerial mySerial(6, 7);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)

// Mux pins
const uint8_t MUX_LSB = 2;
const uint8_t MUX_BIT1 = 3;
const uint8_t MUX_BIT2 = 4;
const uint8_t MUX_MSB = 5;

// LIDAR Count and other wacky constants
const uint8_t LIDAR_COUNT = 1;
const uint8_t US_COUNT = 0; // Ultrasonic sensor count

TFMini tfmini; // The sensor object.

uint16_t distances[LIDAR_COUNT + US_COUNT]; // Measured distances.

void setMux(uint16_t value)
{
  digitalWrite(MUX_LSB, value & 0x01); // Write LSB
  digitalWrite(MUX_BIT1, value & 0x02); // Write Bit 1
  digitalWrite(MUX_BIT2, value & 0x04); // Write Bit 2
  digitalWrite(MUX_MSB, value & 0x08); // Write MSB
}

void setup() 
{
  // Step 1: Initialize hardware serial port (serial debug port)
  Serial.begin(115200);
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial);
     
  Serial.println ("Initializing...");

  // Step 2: Initialize the data rate for the SoftwareSerial port
  mySerial.begin(TFMINI_BAUDRATE);

  // Step 3: Initialize the TF Mini sensors
  // Assumes ultrasonic sensor comes after all LIDAR sensors (can be modified in software)
  for (uint8_t sensorNum = 0; sensorNum < LIDAR_COUNT; ++sensorNum)
  {
    setMux(sensorNum);
    tfmini.begin(&mySerial);
  }
}

void loop() 
{
  static uint8_t currentSensor = 0; // The current sensor being measured.
  
  // Set mux for current sensor.
  setMux(currentSensor);

  Serial.println("Getting measurement from sensor 0");
  // Take LIDAR distance measurement
  uint16_t dist = distances[currentSensor] = tfmini.getDistance();

/*  foreach (motor in motorArray)
  {
    if (isAssociated(motor, currentSensor)) 
    {
        updateMotorPower(motor, distance, currentSensor);
    }
    if (motor.isPulsing)
    {
        if (motor.counter >= pulsePeriod)
        {
            motor.on = !motor.on;
            motor.counter = 0;
        }
        else
        {
            motor.counter++;
        }
    }
  }
*/
  
  if (dist == 65535)
    dist = 0;
  //uint16_t strength = tfmini.getRecentSignalStrength();

  // Display the measurement
  Serial.print(dist);
  Serial.println(" cm");
  //Serial.println(strength);

  // Wait some short time before taking the next measurement
  delay(25);  
}
