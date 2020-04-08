#ifndef _XBOXCONTROLLER_H_
#define _XBOXCONTROLLER_H_

struct xboxControllerInputs
{
    float LJ_LEFT_RIGHT; // (-1.00 - 1.00) Default: 0.00
    float LJ_DOWN_UP;    // (-1.00 - 1.00) Default: 0.00
    float LT;            // (-1.00 - 1.00) Default: -1.00
    float RJ_LEFT_RIGHT; // (0.00 - 1.00) Default: 0.50
    float RJ_DOWN_UP;    // (-1.00 - 1.00) Default: 0.00
    float RT;            // (-1.00 - 1.00) Default: -1.00
    bool A;              // (0 - 1) Default: 0
    bool B;              // (0 - 1) Default: 0
    bool X;              // (0 - 1) Default: 0
    bool Y;              // (0 - 1) Default: 0
    bool LB;             // (0 - 1) Default: 0
    bool RB;             // (0 - 1) Default: 0
    bool MLB;            // (0 - 1) Default: 0
    bool MRB;            // (0 - 1) Default: 0
    bool MB;             // (0 - 1) Default: 0
    bool LJ;             // (0 - 1) Default: 0
    bool RJ;             // (0 - 1) Default: 0
};

struct xboxControllerInputs XBOX_CONTROLLER_INPUT;

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
    if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT != 0)
    {
        if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT < 0)
        {
            manualParams.step_left = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, -1, 0, 10, 160);
        }
        else if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT > 0)
        {
            manualParams.step_right = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, 0, 1, 160, 10);
        }
    }
    else
    {
        manualParams.step_left = 160.0;
        manualParams.step_right = 160.0;
    }
    if (XBOX_CONTROLLER_INPUT.RJ_DOWN_UP != 0)
    {
        manualParams.height += map(XBOX_CONTROLLER_INPUT.RJ_DOWN_UP, -1, 1, -1, 1);
        manualParams.height = constrain(manualParams.height, (80 + manualParams.amplitude_over), (249 - manualParams.amplitude_under));
    }
}

#endif // _XBOXCONTROLLER_H_
