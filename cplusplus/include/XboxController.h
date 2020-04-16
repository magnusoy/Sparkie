#ifndef _XBOXCONTROLLER_H_
#define _XBOXCONTROLLER_H_

#include "IO.h"

bool oldStateA = false;
bool oldStateB = false;
bool oldStateX = false;
bool oldStateY = false;

/**
 * Checks if xbox buttons are pressed
 */
void readXboxButtons()
{
    if (!oldStateB && XBOX_CONTROLLER_INPUT.B)
    {
        if (currentState == S_MANUAL)
        {
            changeStateTo(S_TRANSITION);
        }
        else
        {
            changeStateTo(S_IDLE);
        }
    }
    oldStateB = XBOX_CONTROLLER_INPUT.B;

    if (!oldStateA && XBOX_CONTROLLER_INPUT.A)
    {
        if (currentState == S_IDLE)
        {
            changeStateTo(S_TRANSITION);
        }
        else if (currentState == S_STAND)
        {
            changeStateTo(S_TRANSITIONWALK);
            nextState = S_MANUAL;
        }
    }
    oldStateA = XBOX_CONTROLLER_INPUT.A;

    if (!oldStateY && XBOX_CONTROLLER_INPUT.Y)
    {
        manualParams.height = 170.0;
    }
    oldStateY = XBOX_CONTROLLER_INPUT.Y;
}

/**
 * Maps the diffrent xbox inputs to moved controll
 */
void mapXboxInputs()
{
    val = map(XBOX_CONTROLLER_INPUT.LJ_DOWN_UP, -1, 1, -PI / 55, PI / 55);
    val = constrain(val, -PI / 55, PI / 55);
    if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT > 0.1 || XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT < -0.1)
    {
        if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT < 0)
        {
            manualParams.step_left = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, -1, 0, 0, 80);
        }
        else if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT > 0)
        {
            manualParams.step_right = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, 0, 1, 80, 0);
        }
    }
    else
    {
        manualParams.step_left = 80.0;
        manualParams.step_right = 80.0;
    }
    if (XBOX_CONTROLLER_INPUT.RJ_DOWN_UP != 1)
    {
        manualParams.height += map(XBOX_CONTROLLER_INPUT.RT, 1, -1, 0, 0.0005);
        manualParams.height += map(XBOX_CONTROLLER_INPUT.LT, 1, -1, 0, -0.0005);
        manualParams.height = constrain(manualParams.height, (80 + manualParams.amplitude_over), (249 - manualParams.amplitude_under));
    }
}

#endif // _XBOXCONTROLLER_H_
