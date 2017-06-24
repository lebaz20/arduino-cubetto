#include <Servo.h> 

int analogInputsPins[] = {A0, A1, A2, A3};
int analogInputsPinsForFunction[] = {A4, A5};
int stepAngle = 90;
int stepDelay = 1000;
float minVoltageForLeftMove = 2.05;
float maxVoltageForLeftMove = 2.85;
float minVoltageForRightMove = 1.05;
float maxVoltageForRightMove = 1.85;
float minVoltageForForwardMove = 3.05;
float maxVoltageForForwardMove = 3.85;
float minVoltageForFunction = 4.05;
float maxVoltageForFunction = 4.85;
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

  // loop from the lowest pin to the highest:
  for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPins) / sizeof(int); analogPinKey++) {
    readInputAndPrepareMovements(analogInputsPins[analogPinKey], true);
  }
}

void readInputAndPrepareMovements (int analogInputPin, bool canRunFunction) {
  // read the input on analog pin:
    int sensorValue = analogRead(analogInputPin);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float newVoltage = sensorValue * (5.0 / 1023.0);
    Serial.println(newVoltage);
  
    prepareLeftMove(newVoltage);
    
    prepareRightMove(newVoltage);
  
    prepareForwardMove(newVoltage);

    // normal pins can run functions
    // function pins can not run functions -- infinite loop hazard
    if (canRunFunction == true) {
      prepareFunctionMove(newVoltage);
    }
    
    delay(stepDelay);
}

void prepareFunctionMove(float newVoltage) {
  if (inRange(minVoltageForFunction, maxVoltageForFunction, oldVoltage, newVoltage)) {
    Serial.println("do function move");
    // loop from the lowest function pin to the highest:
    for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPinsForFunction) / sizeof(int); analogPinKey++) {
      readInputAndPrepareMovements(analogInputsPinsForFunction[analogPinKey], false);
    }
  }
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
  delay(stepDelay);
  motor.write(stepAngle);
  delay(stepDelay);
}

void stopMoving(float newVoltage) {
  // update old voltage to do only one step
  oldVoltage = newVoltage;
}

