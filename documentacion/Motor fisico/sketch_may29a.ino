#include <Stepper.h>
#include <EEPROM.h>

const int stepsPerRevolution = 2048; 
const int revolutions = 3;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);
const int eepromAddr = 0;

void setup() {
  myStepper.setSpeed(10);  

  int lastState = EEPROM.read(eepromAddr);

  if (lastState == 1) {
    myStepper.step(-stepsPerRevolution * revolutions);  
    EEPROM.write(eepromAddr, 0);
  } else {
    myStepper.step(stepsPerRevolution * revolutions);
    EEPROM.write(eepromAddr, 1);
  }

  delay(500);
}

void loop() {
  // nothing
}
