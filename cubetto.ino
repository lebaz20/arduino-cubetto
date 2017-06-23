#include <Servo.h> 

int stepAngle = 90;
float minVoltageForLeftMove = 2.05;
float maxVoltageForLeftMove = 2.85;
float minVoltageForRightMove = 1.05;
float maxVoltageForRightMove = 1.85;
float minVoltageForForwardMove = 3.05;
float maxVoltageForForwardMove = 3.85;
int servo1Pin = 6;
int servo2Pin = 9;

float oldVoltage;
Servo servo1; 
Servo servo2;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  servo1.attach(servo1Pin);  // attaches the servo to the servo object 
  servo2.attach(servo2Pin);  // attaches the servo to the servo object 

}

void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float newVoltage = sensorValue * (5.0 / 1023.0);
  Serial.println(newVoltage);

  prepareLeftMove(newVoltage);
  
  prepareRightMove(newVoltage);

  prepareForwardMove(newVoltage);
}

void prepareForwardMove(float newVoltage) {
  if (inRange(minVoltageForForwardMove, maxVoltageForForwardMove, oldVoltage, newVoltage)) {
    Serial.println("do forward move");
    doStep(servo1);
    doStep(servo2);
    stopMoving(newVoltage);
  }
}

void prepareLeftMove(float newVoltage) {
  if (inRange(minVoltageForLeftMove, maxVoltageForLeftMove, oldVoltage, newVoltage)) {
    Serial.println("do left move");
    doStep(servo2);
    stopMoving(newVoltage);
  }
}

void prepareRightMove(float newVoltage) {
  if (inRange(minVoltageForRightMove, maxVoltageForRightMove, oldVoltage, newVoltage)) {
    Serial.println("do right move");
    doStep(servo1);
    stopMoving(newVoltage);
  }
}

bool inRange(float minValue, float maxValue, float oldValue, float currentValue) {
  // old value not in range, while current value is in range
  // in range means less than max and more than min
  return  (! (oldValue < maxValue && oldValue > minValue) && currentValue < maxValue && currentValue > minValue);
}

void doStep(Servo motor) {
  Serial.println("do step");
  // move a step
  motor.write(0);
  delay(1000);
  motor.write(stepAngle);
  delay(1000);
}

void stopMoving(float newVoltage) {
  // update old voltage to do only one step
  oldVoltage = newVoltage;
}

