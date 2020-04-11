#ifndef IO_H_
#define IO_H_

#include "../lib/ButtonTimer/src/ButtonTimer.h"
#include "Locomotion.h"
#include "Globals.h"

/*  Variables used for blinking a led without delay*/
uint8_t ledState = LOW;
unsigned long previousMillis = 0;
const long INTERVAL = 1000;

#define TIME_DELAY 50

ButtonTimer TON1(TIME_DELAY);
ButtonTimer TON2(TIME_DELAY);
ButtonTimer TON3(TIME_DELAY);
ButtonTimer TON4(TIME_DELAY);

/** Initialize switches to inputs. */
void initializeButtons()
{
  pinMode(RED_BTN, INPUT_PULLDOWN);
  pinMode(BLUE_BTN, INPUT_PULLDOWN);
  pinMode(GREEN_BTN, INPUT_PULLDOWN);
  pinMode(ORANGE_BTN, INPUT_PULLDOWN);
}

/** Initialize LEDs to outputs */
void initializeLights()
{
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(ORANGE_LED, OUTPUT);
}

/** Turn off all LEDs */
void turnOffAllLights()
{
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(ORANGE_LED, LOW);
}

/**
   Blinking one LED at a desired pace
   @param pin, The pin to blink
*/
void blinkLight(uint8_t pin)
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL)
  {
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(pin, ledState);
    previousMillis = currentMillis;
  }
}

/**
 * Makes the system settings ready for next state
 */
void setSettings(uint8_t newState)
{
  if (currentState == S_IDLE && (newState == S_STAND || newState == S_TRANSITION))
  {
    armMotors();
  }
  else if ((currentState == S_STAND || currentState == S_TRANSITION || currentState == S_RESET) && newState == S_IDLE)
  {
    disarmMotors();
  }

  if (currentState == S_TRANSITION)
  {
    transition = false;
  }

  if (newState == S_CALIBRATE)
  {
    digitalWrite(BLUE_LED, HIGH);
  }

  if (nextState == S_STAND)
  {
    pitchPID.reset();
    rollPID.reset();
    yawPID.reset();
  }

  if (newState == S_WALK || newState == S_MANUAL || newState == S_AUTONOMOUS)
  {

    setLegMotorTrapTraj(50000, 50000, 50000);
    pitchPID.reset();
    rollPID.reset();
    yawPID.reset();
    digitalWrite(GREEN_LED, HIGH);
    setLegMotorTrapTraj(50000, 50000, 50000);
  }

  if (currentState == S_WALK || currentState == S_MANUAL || currentState == S_AUTONOMOUS)
  {
    setLegMotorTrapTraj(500, 500, 500);
    val = PI / 140;
    autoParams.x = 0;
    manualParams.x = 0;
    setLegMotorTrapTraj(500, 500, 500);
  }
}

/**
   Change the state of the statemachine to the new state
   given by the parameter newState
   @param newState The new state to set the statemachine to
*/
void changeStateTo(uint8_t newState)
{
  turnOffAllLights();
  setSettings(newState);
  currentState = newState;
}

/** Reads all the buttons and change the state if a button is pressed */
void readButtons()
{
  if (TON1.isSwitchOn(RED_BTN))
  {
    if (currentState == S_WALK || currentState == S_MANUAL || currentState == S_AUTONOMOUS)
    {
      changeStateTo(S_TRANSITION);
    }
    else
    {
      changeStateTo(S_IDLE);
    }
  }

  else if (TON2.isSwitchOn(ORANGE_BTN))
  {
    changeStateTo(S_RESET);
  }

  else if (TON3.isSwitchOn(BLUE_BTN))
  {
    changeStateTo(S_CALIBRATE);
  }

  else if (TON4.isSwitchOn(GREEN_BTN))
  {
    if (currentState == S_IDLE)
    {
      changeStateTo(S_TRANSITION);
    }
    else if (currentState == S_STAND)
    {
      changeStateTo(S_TRANSITIONWALK); //Change to S_LAYDOWN for laydown  test
      nextState = S_WALK;
    }
    else if (currentState == S_LAYDOWN)
    {
      changeStateTo(S_TRANSITION);
    }
  }
}
#endif // IO_H_
