#include <ButtonTimer.h>

// Defining limit switches
const int LIMIT_SWITCH_Y_BOTTOM = 2;
const int LIMIT_SWITCH_Y_TOP = 3;

#define TIME_DELAY 500;

ButtonTimer TON1(TIME_DELAY);
ButtonTimer TON2(TIME_DELAY);

void setup() {
  Serial.begin(9600);
  initializeSwitches();
}

void loop() {
  if (TON1.isSwitchOn(LIMIT_SWITCH_Y_BOTTOM)) {
    Serial.println("Button 1 Pressed");
  }
  if (TON2.isSwitchOn(LIMIT_SWITCH_Y_TOP)) {
    Serial.println("Button 2 Pressed");
  }
}

/**
  Initialize limit switches to inputs.
*/
void initializeSwitches() {
  pinMode(LIMIT_SWITCH_Y_BOTTOM, INPUT);
  pinMode(LIMIT_SWITCH_Y_TOP, INPUT);
}