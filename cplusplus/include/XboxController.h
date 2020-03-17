#ifndef _XBOXCONTROLLER_H_
#define _XBOXCONTROLLER_H_


struct xboxControllerInputs {
    float LJ_LEFT_RIGHT; // (-100.00 - 100.00) Default: 0.00
    float LJ_DOWN_UP; // (-100.00 - 100.00) Default: 0.00
    float LT; // (-100.00 - 100.00) Default: -100.00
    float RJ_LEFT_RIGHT; // (-100.00 - 100.00) Default: 0.00
    float RJ_DOWN_UP; // (-100.00 - 100.00) Default: 0.00
    float RT; // (-100.00 - 100.00) Default: -100.00
    bool A; // (0 - 1) Default: 0
    bool B; // (0 - 1) Default: 0
    bool X; // (0 - 1) Default: 0
    bool Y; // (0 - 1) Default: 0
    bool LB; // (0 - 1) Default: 0
    bool RB; // (0 - 1) Default: 0
    bool MLB; // (0 - 1) Default: 0
    bool MRB; // (0 - 1) Default: 0
    bool MB; // (0 - 1) Default: 0
    bool LJ; // (0 - 1) Default: 0
    bool RJ; // (0 - 1) Default: 0
};

struct xboxControllerInputs XBOX_CONTROLLER_INPUT;


#endif // _XBOXCONTROLLER_H_

