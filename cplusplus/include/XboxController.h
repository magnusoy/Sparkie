#ifndef _XBOXCONTROLLER_H_
#define _XBOXCONTROLLER_H_

struct xboxControllerInputs
{
    float LJ_LEFT_RIGHT; // (-100.00 - 100.00) Default: 0.00
    float LJ_DOWN_UP;    // (-100.00 - 100.00) Default: 0.00
    float LT;            // (-100.00 - 100.00) Default: -100.00
    float RJ_LEFT_RIGHT; // (0.00 - 100.00) Default: 50.00
    float RJ_DOWN_UP;    // (-100.00 - 100.00) Default: 0.00
    float RT;            // (-100.00 - 100.00) Default: -100.00
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

/**
 * Checks if xbox buttons are pressed
 */
void readXboxButtons()
{
    if (XBOX_CONTROLLER_INPUT.B == 1)
    {
        disarmMotors();
        changeStateTo(S_IDLE);
    }

    if (XBOX_CONTROLLER_INPUT.Y == 1)
    {
        manualParams->height = 170.0;
    }
}

/**
 * Maps the diffrent xbox inputs to moved controll
 */
void mapXboxInputs()
{
    val = map(XBOX_CONTROLLER_INPUT.LJ_DOWN_UP, -100, 100, -PI / 55, PI / 55);
    val = constrain(val, -PI / 55, PI / 55);
    if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT <= 45 || XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT >= 55)
    {
        if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT < 45)
        {
            manualParams->step_left = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, 0, 44, 10, 160);
        }
        else if (XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT > 55)
        {
            manualParams->step_right = map(XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT, 56, 100, 160, 10);
        }
    }
    else
    {
        manualParams->step_left = 160.0;
        manualParams->step_right = 160.0;
    }
    if (XBOX_CONTROLLER_INPUT.RJ_DOWN_UP != 0)
    {
        manualParams->height += map(XBOX_CONTROLLER_INPUT.RJ_DOWN_UP, -100, 100, -1, 1);
        manualParams->height = constrain(manualParams->height, (80 + manualParams->amplitude_over), (249 - manualParams->amplitude_under));
    }
}

#endif // _XBOXCONTROLLER_H_
