// Sensor control program for Blindsighted. Ver. 1.00, 1/21/2020
// Written in collaboration by Garrisen Cizmich and Jonah Cullen, including libraries created by Peter Jansen and Adafruit.
// Reads distance inputs from each sensor and adjusts motor response accordingly.
// Note: If hardware is adjusted in any way (due to sensor or motor changes), you will have to change some of this code.
// Most things that you need to change will be marked by the keyword "HW". Also look in Motor.h and Motor.cpp for this keyword.

/* Pins:
  0 - US1 Trig (11 on DSub)
  1 - US1 Echo (12 on Dsub)
  2 - Mux LSB (bit 0)
  3 - Mux bit 1
  4 - Mux bit 2
  5 - Mux MSB (bit 3)
  6 - LIDAR TX (Arduino RX)
  7 - LIDAR RX (Arduino TX)
  8 - US2 Trig (13 on DSub)
  9 - US2 Echo (14 on DSub)
  10 - SD Card Reader CS
  11 - SD Card Reader MOSI
  12 - SD Card Reader MISO
  13 - SD Card Reader SCK
  SDA - Motor Driver SDA
  SCL - Motor Driver SCL

  A1 -
  A2 -
  A3 -
  A4 -
  A5 -

*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include "Motor.h"

// Switch for enabling debug mode.
const bool DEBUG_MODE = false;

// Create the PWM object for the servo driver using the default address (0x40).
// The maximum pulse length is 4096 (0 - 4095).
Adafruit_PWMServoDriver pwm; // The object for interfacing with the PWM driver.

// Setup software serial port
SoftwareSerial mySerial(6, 7); // Uno RX (TFMINI TX), Uno TX (TFMINI RX)

// Mux pins
const uint8_t MUX_LSB = 2;
const uint8_t MUX_BIT1 = 3;
const uint8_t MUX_BIT2 = 4;
const uint8_t MUX_MSB = 5;

// Ultrasonic pins
const uint8_t US1_TRIG = 0;
const uint8_t US1_ECHO = 1;
const uint8_t US2_TRIG = 8;
const uint8_t US2_ECHO = 9;

// HW: Sensor/motor counts
const uint8_t LIDAR_COUNT = 3; // Number of LIDAR sensors.
const uint8_t US_COUNT = 0; // Number of Ultrasonic sensors.
const uint8_t MOTOR_COUNT = 8; // Number of haptic motors.

// HW: Array of actual LIDAR mux values (e.g. sensor 0 might correspond to mux value 2, so sensorMuxValues[0] would be 2).
const uint8_t sensorMuxValues[] = {2, 1, 0}; // The number of terms here must match the value of LIDAR_COUNT.

// Motor array.
Motor motorArray[MOTOR_COUNT];

// HW: Array of actual motor numbers (based on the ports used with the PWM Motor Driver).
const uint8_t motorPortValues[] = {0, 1, 2, 3, 4, 5, 6, 7}; // The number of terms here must match the value of MOTOR_COUNT.

// HW: For the current design, we want to associate sensor 0 with motors 3 and 4, sensor 1 with motors 1 and 2, and sensor 2 with motor 0.
// We also want to associate the ultrasonic sensor (sensor 3) with motor 6, but we are disabling it temporarily. Motor 7 is present but unused. 0 indicates a sensor has no effect on a motor.
// The first index is the motor, the second is the sensor. 
// The inner array should have the same number of entries as the total sensor count. The outer array should have the same number of inner arrays as there are motors.
// The weights are integers to conserve space. You must multiply the weight you desire by 255 and round down to get the weight to store.
const uint8_t sensorWeights[][LIDAR_COUNT + US_COUNT] = {{0, 0, 255},  // 0
                                                        {0, 255, 0},   // 1
                                                        {0, 255, 0},   // 2
                                                        {255, 0, 0},   // 3
                                                        {255, 0, 0},   // 4
                                                        {0, 0, 0},   // 5
                                                        {0, 0, 0},   // 6
                                                        {0, 0, 0}};    // 7

TFMini tfmini; // The sensor object.

uint16_t distances[LIDAR_COUNT + US_COUNT]; // Measured distances in cm. Ultrasonic measurements ALWAYS come after LIDAR measurements in the array.

File dataFile; // The data file to be stored on the SD card.

// Sets the mux to the desired value.
void setMux(uint8_t value)
{
  digitalWrite(MUX_LSB, value & 0x01); // Write LSB
  digitalWrite(MUX_BIT1, value & 0x02); // Write Bit 1
  digitalWrite(MUX_BIT2, value & 0x04); // Write Bit 2
  digitalWrite(MUX_MSB, value & 0x08); // Write MSB
}

void setup()
{
  // Initialize hardware serial port (serial debug port). Not necessary for use on final design.
  // May cause issues if trying to use with Ultrasonic sensor on D0 and D1.
  // Uses 115200 baud (TFmini baud) for fun.
  if (DEBUG_MODE) Serial.begin(TFMINI_BAUDRATE);
  
  // Set up the PWM motor driver for motor control.
  if (DEBUG_MODE) Serial.println("Readying PWM");
  pwm = Adafruit_PWMServoDriver();
  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000); // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(50); // The frequency in Hz of the PWM signal sent to the motors. The frequency is not too important, but it needs to be high enough to "trick" the motor.
  
  delay(10);

  // Initialize the data rate for the SoftwareSerial port (for LIDAR comms).
  if (DEBUG_MODE) Serial.println("Readying SoftSerial");
  mySerial.begin(TFMINI_BAUDRATE);

  // Set the mux pins for output mode (very important!).
  if (DEBUG_MODE) Serial.println("Setting pin modes");
  pinMode(MUX_LSB, OUTPUT);
  pinMode(MUX_BIT1, OUTPUT);
  pinMode(MUX_BIT2, OUTPUT);
  pinMode(MUX_MSB, OUTPUT);

  // Set the Ultrasonic pins for input/output mode as needed.
  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);

  // Initialize the motor objects in the array of motors.
  if (DEBUG_MODE) Serial.println("Readying motors");
  for (uint8_t motorNum = 0; motorNum < MOTOR_COUNT; ++motorNum)
  {
    motorArray[motorNum] = Motor(motorPortValues[motorNum], sensorWeights[motorNum]);
  }

  // Initialize the TF Mini sensors.
  for (uint8_t sensorNum = 0; sensorNum < LIDAR_COUNT; ++sensorNum)
  {
    if (DEBUG_MODE)
    {
      Serial.print("Readying LIDAR sensor ");
      Serial.println(sensorNum);
    }
    
    setMux(sensorMuxValues[sensorNum]); // Set the mux so that we can access the sensor.
    if (!tfmini.begin(&mySerial))
    {
      if (DEBUG_MODE) Serial.println("Readying failed");
    }
    tfmini.setSingleScanMode();
  }
  
  // Initialize the distance array to have all zeros.
  for (uint8_t sensorNum = 0; sensorNum < LIDAR_COUNT + US_COUNT; ++sensorNum)
  {
    distances[sensorNum] = 0;
  }

  // Initialize the SD card reader.
  if (DEBUG_MODE) Serial.println("Readying SD card");
  pinMode(10, OUTPUT); // Set pin 10 to be an output.
  if (!SD.begin(10))
  {
    if (DEBUG_MODE) Serial.println("Readying failed");
  }

  // Open the data file and write the flag.
  if (DEBUG_MODE) Serial.println("Opening file");
  dataFile = SD.open("data.txt", FILE_WRITE);
  
  if (DEBUG_MODE) Serial.println("Writing flag");
  dataFile.write("BGN\n");
}

void loop()
{

  static uint8_t currentSensor = 0; // The current sensor being measured.
  static uint32_t updateCount = 0; // Counter for numbering samples.
  
  // Get the new distance measurement.
  if (currentSensor < LIDAR_COUNT) // If the sensor is a LIDAR sensor.
  {
    if (DEBUG_MODE)
    {
      Serial.print("Pinging LIDAR ");
      Serial.println(currentSensor);
    }
    // Set mux for current sensor.
    setMux(sensorMuxValues[currentSensor]);
  
    // Take LIDAR distance measurement
    tfmini.externalTrigger();
    uint16_t distance = tfmini.getDistance();
    if (distance <= 12000) // Discard obviously erroneous measurements.
    {
      distances[currentSensor] = distance;
    }
    if (DEBUG_MODE)
    {
      Serial.print(distances[currentSensor]);
      Serial.println(" cm");
    }
  }
  else // The sensor is an ultrasonic sensor.
  {
    // HW: Add code to differentiate between US1 and US2 based on the value of currentSensor and take measurements for each.
    
    /*if (DEBUG_MODE) Serial.println("Pinging US2");
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(US2_TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(US2_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US2_TRIG, LOW);
   
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    uint32_t pulseTime = pulseIn(US2_ECHO, HIGH, 10000); // The pulse time for the measurement.

    if (pulseTime > 0) // Only keep valid measurements.
    {
      // Convert the time into a distance.
      distances[currentSensor] = (pulseTime / 2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    }
    if (DEBUG_MODE)
    {
      Serial.print(distances[currentSensor]);
      Serial.println(" cm");
    }*/
  }

  if (DEBUG_MODE) Serial.println("Updating motors...");
  for (uint8_t motorNum = 0; motorNum < MOTOR_COUNT; ++motorNum)
  {
    motorArray[motorNum].updateMotorPower(distances, LIDAR_COUNT + US_COUNT, pwm);
    if (motorArray[motorNum].getIsPulsing())
    {
        if (motorArray[motorNum].getCounter() >= PULSE_PERIOD)
        {
            motorArray[motorNum].setMotorVal(4095 - motorArray[motorNum].getMotorVal(), pwm);
            motorArray[motorNum].setCounter(0);
        }
    }
  }
  
  // Display the measurement.
  if (DEBUG_MODE)
  {
    Serial.print(updateCount);
    Serial.print(',');
    Serial.print(currentSensor);
    Serial.print(',');
    Serial.print(distances[currentSensor]);
    for (int motorNum = 0; motorNum < MOTOR_COUNT; ++motorNum)
    {
      Serial.print(',');
      Serial.print(motorNum);
      Serial.print(',');
      Serial.print(motorArray[motorNum].getMotorVal());
    }
    Serial.println();
  }

  // Write the measurement to file and flush it to memory.
  char temp[30]; // Char array (string) to store text versions of integer values.
  itoa(updateCount, temp, 10);
  dataFile.write(temp);
  dataFile.write(',');
  itoa(currentSensor, temp, 10);
  dataFile.write(temp);
  dataFile.write(',');
  itoa(distances[currentSensor], temp, 10);
  dataFile.write(temp);
  for (int motorNum = 0; motorNum < MOTOR_COUNT; ++motorNum)
    {
      dataFile.write(',');
      itoa(motorNum, temp, 10);
      dataFile.write(temp);
      dataFile.write(',');
      itoa(motorArray[motorNum].getMotorVal(), temp, 10);
      dataFile.write(temp);
    }
  dataFile.write('\n');
  dataFile.flush();
  ++updateCount;

  // Switch to the next sensor.
  ++currentSensor;
  currentSensor %= LIDAR_COUNT + US_COUNT;

  // Wait some short time before taking the next measurement
  delay(25);
}
