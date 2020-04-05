#ifndef _BUTTONTIMER_H_
#define _BUTTONTIMER_H_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class ButtonTimer
{

public:
    // Defining Functions
    ButtonTimer(unsigned long timeOut);
    bool isSwitchOn(int btn);
    bool buttonTimerHasExpired();
    void startButtonTimer(unsigned long duration);

private:
    // Defining varibales
    unsigned long nextButtonTimeout;
    unsigned long timeOut;
    bool running;
    int oldBtnState;
    bool result;
};
#endif // _BUTTONTIMER_H_
