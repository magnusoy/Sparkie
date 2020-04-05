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
   Change the state of the statemachine to the new state
   given by the parameter newState
   @param newState The new state to set the statemachine to
*/
void changeStateTo(uint8_t newState)
{
  turnOffAllLights();
  idlePosition = false;
  currentState = newState;
}

/** Reads all the buttons and change the state if a button is pressed */
void readButtons()
{
  if (TON1.isSwitchOn(RED_BTN))
  {
    if (currentState == S_STAND)
    {
      disarmMotors();
      changeStateTo(S_IDLE);
    }
    else
    {
      changeStateTo(S_STAND);
    }
  }
  else if (TON2.isSwitchOn(ORANGE_BTN))
  {
    changeStateTo(S_RESET);
  }
  else if (TON3.isSwitchOn(BLUE_BTN))
  {
    changeStateTo(S_CALIBRATE);
    digitalWrite(BLUE_LED, HIGH);
  }
  else if (TON4.isSwitchOn(GREEN_BTN))
  {
    if (currentState == S_IDLE)
    {
      armMotors();
      changeStateTo(S_STAND);
    }

    else
    {
      changeStateTo(S_WALK);
      digitalWrite(GREEN_LED, HIGH);
      //setLegMotorPID(25.0f, 0.001f, 0.0005f);
    }
  }
}
#endif // IO_H_
