/* Based on code from Rui Santos, https://randomnerdtutorials.com
   Written by Garrisen Cizmich for use in the Blind-Sighted Senior Project with NNU.
   Cycles through four ultrasonic sensors and reports the perceived distance through haptic feedback motors connected via I2C.
*/
const int numberOfSensors = 4;
const int selectorPin0 = 4; // LSB for the selector
const int selectorPin1 = 5; // Position 1 for the selector
const int selectorPin2 = 6; // Position 2 for the selector
const int selectorPin3 = 7; // MSB for the selector
const int trigPin = 11;     // Trigger
const int echoPin = 12;     // Echo
long duration, cm;
short int sensorNum = 0;

void setup()
{
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(selectorPin0, OUTPUT);
  pinMode(selectorPin1, OUTPUT);
  pinMode(selectorPin2, OUTPUT);
  pinMode(selectorPin3, OUTPUT);
  digitalWrite(selectorPin0, LOW);
  digitalWrite(selectorPin1, LOW);
  digitalWrite(selectorPin2, LOW);
  digitalWrite(selectorPin3, LOW);
}

void loop()
{
  if (sensorNum & 0x01)
  {
    digitalWrite(selectorPin0, HIGH);
  }
  else
  {
    digitalWrite(selectorPin0, LOW);
  }

  if (sensorNum & 0x02)
  {
    digitalWrite(selectorPin1, HIGH);
  }
  else
  {
    digitalWrite(selectorPin1, LOW);
  }

  if (sensorNum & 0x04)
  {
    digitalWrite(selectorPin2, HIGH);
  }
  else
  {
    digitalWrite(selectorPin2, LOW);
  }

  if (sensorNum & 0x08)
  {
    digitalWrite(selectorPin3, HIGH);
  }
  else
  {
    digitalWrite(selectorPin3, LOW);
  }

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH, 25000);

  // Convert the time into a distance
  cm = (duration / 2) / 29.1;   // Divide by 29.1 or multiply by 0.0343

  if (sensorNum == 3)
  {
    Serial.print("Sensor,");
    Serial.print(sensorNum);
    Serial.print(",");
    Serial.print(cm);
    Serial.print(",cm");
    Serial.println();
  }

  //delay(63);
  sensorNum = ++sensorNum % numberOfSensors;
}
