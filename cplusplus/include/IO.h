#ifndef _IO_H_
#define _IO_H_

/*  Variables used for blinking a led without delay*/
uint8_t ledState = LOW;
unsigned long previousMillis = 0;
const long INTERVAL = 1000;

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
  uint8_t green = digitalRead(GREEN_BTN);
  uint8_t red = digitalRead(RED_BTN);
  uint8_t blue = digitalRead(BLUE_BTN);
  uint8_t orange = digitalRead(ORANGE_BTN);
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
