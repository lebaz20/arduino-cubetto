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

float oldVoltage[10];
Servo servo1; 
Servo servo2;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  servo1.attach(servo1Pin);  // attaches the servo to the servo object 
  servo2.attach(servo2Pin);  // attaches the servo to the servo object 

}

void loop() {

  if (shouldReadInputs()) {
    // loop from the lowest pin to the highest:
    for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPins) / sizeof(int); analogPinKey++) {
      readInputAndPrepareMovements(analogInputsPins[analogPinKey], true);
    }
  }
  prepareResetListener();
}

void readInputAndPrepareMovements (int analogInputPin, bool canRunFunction) {
  // read the input on analog pin:
    int sensorValue = analogRead(analogInputPin);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float newVoltage = sensorValue * (5.0 / 1023.0);
    Serial.println(newVoltage);
  
    prepareLeftMoveListener(newVoltage, analogInputPin);
    
    prepareRightMoveListener(newVoltage, analogInputPin);
  
    prepareForwardMoveListener(newVoltage, analogInputPin);

    // normal pins can run functions
    // function pins can not run functions -- infinite loop hazard
    if (canRunFunction == true) {
      prepareFunctionMoveListener(newVoltage, analogInputPin);
    }
    
    delay(stepDelay);
}

void prepareFunctionMoveListener(float newVoltage, int analogInputPin) {
  if (isInRange(minVoltageForFunction, maxVoltageForFunction, oldVoltage[analogInputPin], newVoltage)) {
    Serial.println("do function move");
    // loop from the lowest function pin to the highest:
    for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPinsForFunction) / sizeof(int); analogPinKey++) {
      // reset function movements to be able to trigger them whenever called
      oldVoltage[analogInputsPinsForFunction[analogPinKey]] = 0;
      
      readInputAndPrepareMovements(analogInputsPinsForFunction[analogPinKey], false);
    }
  }
}

void prepareForwardMoveListener(float newVoltage, int analogInputPin) {
  if (isInRange(minVoltageForForwardMove, maxVoltageForForwardMove, oldVoltage[analogInputPin], newVoltage)) {
    Serial.println("do forward move");
    Servo motors[] = {servo1, servo2};
    doStep(motors);
    stopMoving(newVoltage, analogInputPin);
  }
}

void prepareLeftMoveListener(float newVoltage, int analogInputPin) {
  if (isInRange(minVoltageForLeftMove, maxVoltageForLeftMove, oldVoltage[analogInputPin], newVoltage)) {
    Serial.println("do left move");
    Servo motors[] = {servo2};
    doStep(motors);
    stopMoving(newVoltage, analogInputPin);
  }
}

void prepareRightMoveListener(float newVoltage, int analogInputPin) {
  if (isInRange(minVoltageForRightMove, maxVoltageForRightMove, oldVoltage[analogInputPin], newVoltage)) {
    Serial.println("do right move");
    Servo motors[] = {servo1};
    doStep(motors);
    stopMoving(newVoltage, analogInputPin);
  }
}

void prepareResetListener() {
  // reset only if all analog inputs are holding zeros or digital low
  bool shouldReset = true;
  // loop from the lowest pin to the highest:
  for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPins) / sizeof(int); analogPinKey++) {
    if (digitalRead(analogInputsPins[analogPinKey]) != LOW) {
      shouldReset = false;
    }
  }

  if (shouldReset == true) {
    // loop from the lowest pin to the highest:
    for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPins) / sizeof(int); analogPinKey++) {
      oldVoltage[analogPinKey] = 0;
    }
  }
}

bool isInRange(float minValue, float maxValue, float oldValue, float currentValue) {
  // old value not in range, while current value is in range
  // in range means less than max and more than min
  return (! (oldValue < maxValue && oldValue > minValue) && currentValue < maxValue && currentValue > minValue);
}

bool shouldReadInputs() {
  // read from inputs only if old voltages are all set to 0
  bool shouldReadInputs = true;
  // loop from the lowest pin to the highest:
  for (int analogPinKey = 0; analogPinKey < sizeof(analogInputsPins) / sizeof(int); analogPinKey++) {
    if (oldVoltage[analogPinKey] != 0) {
      shouldReadInputs = false;
    }
  }
  return shouldReadInputs;
}

void doStep(Servo motors[]) {
  Serial.println("do step");
  // move a step
  int angles[] = {0, stepAngle};
  for (int angleKey = 0; angleKey < sizeof(angles) / sizeof(int); angleKey++) {
    // loop on all motors to set angle
    for (int motorKey = 0; motorKey < sizeof(motors) / sizeof(Servo); motorKey++) {
      motors[motorKey].write(angles[angleKey]);
    }
    delay(stepDelay);
  }
}

void stopMoving(float newVoltage, int analogInputPin) {
  // update old voltage to do only one step
  oldVoltage[analogInputPin] = newVoltage;
}

