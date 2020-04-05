#include "ButtonTimer.h"

/**
   Initializes the button timer. Assigns the
   given timout in milliseconds.

   @param timeOut time until button confirmation 
  
 */
ButtonTimer::ButtonTimer(unsigned long timeOut)
{
    this->timeOut = timeOut;
}

/**
   Starts the timer and set the timer to expire after the
   number of milliseconds given by the parameter duration.

   @param duration The number of milliseconds until the timer expires.
*/
void ButtonTimer::startButtonTimer(unsigned long duration)
{
    this->nextButtonTimeout = millis() + duration;
}

/**
   Checks if the timer has expired. If the timer has expired,
   true is returned. If the timer has not yet expired,
   false is returned.

   @return true if timer has expired, false if not
*/
bool ButtonTimer::buttonTimerHasExpired()
{
    return (millis() > this->nextButtonTimeout) ? true : false;
}

/**
   Checks if the button is HIGH and
   it stays that for longer than the
   assigned timeout period.

   @param btn button pin to be read
  
 */
bool ButtonTimer::isSwitchOn(uint8_t btn)
{
    uint8_t btnState = digitalRead(btn);
    if (btnState)
    {
        if ((btnState != oldBtnState))
        {
            startButtonTimer(this->timeOut);
            this->result = true;
        }
    }
    oldBtnState = btnState;
    if (buttonTimerHasExpired() && (this->result)) {
        this->result = false;
        return true;
    }
    else {
        return false;
    }
}