// This header file includes the Motor class, which is used to manage information about the motor's functionality. Developed by Garrisen Cizmich on 4/13/2020.
// Through sequential #include statements, we reduce the total amount of memory needed by the program.
#include "TFMini.h"
#include <Adafruit_PWMServoDriver.h>

// HW: Motor response constants.
const uint16_t MIN_DIST = 30; // Minimum distance for response in cm.
const uint16_t PULSE_THRESHOLD = 50; // The maximum distance at which pulsing begins in cm.
const uint16_t MAX_DIST = 300; // The maximum distance at which a response occurs in cm.
const uint8_t PULSE_PERIOD = 3; // The number of loop iterations before the motor state switches while pulsing. Corresponds to a time delay, depending on delay set in loop.

class Motor
{
  public: 
    Motor(void); // Default constructor, does not set portnum.
    Motor(uint8_t portNum, const uint8_t sensorWeights[]);
    void updateMotorPower(uint16_t distance[], uint8_t sensorCount, Adafruit_PWMServoDriver &pwm); // Updates the motor's power based on the new distance information and the sensor number.

    // Setters and getters for the various member variables.
    bool getIsPulsing(); // Tells whether the motor is pulsing.
    uint8_t getCounter(); // Returns the value of counter, then increments it.
    void setCounter(uint8_t counter); // Sets the value of counter.
    uint16_t getMotorVal(); // Gets the current motor value.
    void setMotorVal(uint16_t motorVal, Adafruit_PWMServoDriver &pwm); // Sets the current motor value and updates the PWM driver. Only used to implement pulsing functionality.

  private:

    // Motor variables
    const uint8_t * sensorWeights; // The weighting for each sensor, used to influence how much a motor responds to a given sensor.
    uint8_t portNum; // The actual hardware port number for the motor.
    bool isPulsing; // Whether the motor is pulsing or not.
    uint8_t counter; // The counter for the motor, counts increments of roughly 25 ms.
    uint16_t motorVal; // The PWM upper limit for the motor.
};
