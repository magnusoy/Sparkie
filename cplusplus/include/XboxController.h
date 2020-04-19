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
        manualParams.height = normalHeight;
        pitchSetPoint = 0;
    }
    oldStateY = XBOX_CONTROLLER_INPUT.Y;
}

/**
 * Maps the diffrent xbox inputs to moved controll
 */
void mapXboxInputs()
{
    robotVelocity = map(XBOX_CONTROLLER_INPUT.LJ_DOWN_UP, -1, 1, -maxSpeed, maxSpeed);
    robotVelocity = constrain(robotVelocity, -maxSpeed, maxSpeed);
    if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT > 0.1 || XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT < -0.1)
    {
        if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT < 0)
        {
            manualParams.step_left = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, -1, 0, 0, normalStepLength);
        }
        else if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT > 0)
        {
            manualParams.step_right = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, 0, 1, normalStepLength, 0);
        }
    }
    else
    {
        manualParams.step_left = normalStepLength;
        manualParams.step_right = normalStepLength;
    }
    if (XBOX_CONTROLLER_INPUT.RJ_DOWN_UP != 1)
    {
        manualParams.height += map(XBOX_CONTROLLER_INPUT.RT, 1, -1, 0, 0.0005);
        manualParams.height += map(XBOX_CONTROLLER_INPUT.LT, 1, -1, 0, -0.0005);
        manualParams.height = constrain(manualParams.height, (80 + manualParams.amplitude_over), (249 - manualParams.amplitude_under));
    }

    if (XBOX_CONTROLLER_INPUT.MLB != 0 || XBOX_CONTROLLER_INPUT.MRB != 0)
    {
        pitchSetPoint += map(XBOX_CONTROLLER_INPUT.MRB, 0, 1, 0, 0.05);
        pitchSetPoint += map(XBOX_CONTROLLER_INPUT.MLB, 0, 1, 0, -0.05);
        pitchSetPoint = constrain(pitchSetPoint, -45, 45);
    }
}

#endif // _XBOXCONTROLLER_H_
