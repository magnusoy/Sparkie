#ifndef _IO_H_
#define _IO_H_

/*  Variables used for blinking a led without delay*/
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 1000;

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
void blinkLight(int pin)
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(pin, ledState);
  }
}

/**
   Change the state of the statemachine to the new state
   given by the parameter newState
   @param newState The new state to set the statemachine to
*/
void changeStateTo(int newState)
{
  turnOffAllLights();
  idlePosition = false;
  currentState = newState;
}

/** Reads all the buttons and change the state if a button is pressed */
void readButtons()
{
  int green = digitalRead(GREEN_BTN);
  int red = digitalRead(RED_BTN);
  int blue = digitalRead(BLUE_BTN);
  int orange = digitalRead(ORANGE_BTN);
  if (red)
  {
    disarmMotors();
    changeStateTo(S_IDLE);
  }
  else if (orange)
  {
    changeStateTo(S_RESET);
  }
  else if (blue)
  {
    changeStateTo(S_CALIBRATE);
  }
  else if (green)
  {
    armMotors();
    changeStateTo(S_MANUAL);
    digitalWrite(GREEN_LED, HIGH);
  }
}
#endif // _IO_H_
