// Function implementations for the Motor class within Motor.h. Developed by Garrisen Cizmich on 4/13/2020.
// For details on what each function should do, please view Motor.h.

#include "Motor.h"

Motor::Motor(void)
{
  isPulsing = false;
  counter = 0;
  motorVal = 0;
}

Motor::Motor(uint8_t portNum, const uint8_t sensorWeights[])
{
  this->portNum = portNum;
  this->sensorWeights = sensorWeights;
  isPulsing = false;
  counter = 0;
  motorVal = 0;
}

void Motor::updateMotorPower(uint16_t distance[], uint8_t sensorCount, Adafruit_PWMServoDriver &pwm)
{
  uint16_t weightedMotorVal = 0; // The motor value (after being weighted by the sensor).
  uint16_t unweightedMotorVal = 0; // The unweighted motor value (based only on a single sensor).
  for (uint8_t sensorNum = 0; sensorNum < sensorCount; ++sensorNum)
  {
    if (distance[sensorNum] <= MIN_DIST || distance[sensorNum] > MAX_DIST) // Obstacle is outside the sensor range or the necessary distance range.
    {
      unweightedMotorVal = 0;
      counter = 0;
    }
    else if (distance[sensorNum] > MIN_DIST && distance[sensorNum] <= PULSE_THRESHOLD) // Obstacle is close enough to begin pulsing.
    {
      // Pulsing is a special case as it is less of a value than a mode. 
      // If we need to pulse the motor, we ignore every other sensor's input and return immediately, maintaining the priority of close objects.
      // The only caveat is that we don't pulse if the sensor does not impact the motor.
      if (sensorWeights[sensorNum] > 0)
      {      
        // We run the following code snippet only if we aren't already pulsing as it sets the mode up and will ruin the pulsing functionality if it always runs.
        if (!isPulsing)
        {
          motorVal = 4095; // Turn the motor on full at first.
          counter = 0;
          isPulsing = true;
          pwm.setPWM(portNum, 0, motorVal);
        }
        return;
      }
    }
    else if (distance[sensorNum] > PULSE_THRESHOLD && distance[sensorNum] <= MAX_DIST)
    {
      double slope = 4095 / (double(PULSE_THRESHOLD) - double(MAX_DIST)); // The slope of the proportional relationship between motor value and distance.
      unweightedMotorVal = uint16_t(slope * (distance[sensorNum] - PULSE_THRESHOLD) + 4095); // Calculate the motor value proportionally.
    }
    weightedMotorVal += (unweightedMotorVal / 255.0 * sensorWeights[sensorNum]);
  }

  // If we got to this point, we aren't pulsing the motor. Mark the variable to say as much.
  isPulsing = false;
  motorVal = weightedMotorVal;
  pwm.setPWM(portNum, 0, motorVal);
}

bool Motor::getIsPulsing()
{
  return isPulsing;
}

uint8_t Motor::getCounter()
{
  return counter++;
}

void Motor::setCounter(uint8_t counter)
{
  this->counter = counter;
}
uint16_t Motor::getMotorVal()
{
  return motorVal;
}

void Motor::setMotorVal(uint16_t motorVal, Adafruit_PWMServoDriver &pwm)
{
  this->motorVal = motorVal;
  pwm.setPWM(portNum, 0, motorVal);
}
