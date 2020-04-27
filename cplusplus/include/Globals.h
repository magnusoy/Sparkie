#ifndef GLOBALS_H_
#define GLOBALS_H_
#include "Constants.h"

struct orientation
{
  double pitch;
  double roll;
  double yaw;
};

struct orientation ORIENTAION;

struct position
{
  double x;
  double y;
  double z;
};

struct position POSITION;

/** Error codes */
enum errors
{
  NONE,
  SERIAL_ERROR,
  MOTOR_FRONT_LEFT_1_ERROR,
  MOTOR_FRONT_LEFT_2_ERROR,
  MOTOR_FRONT_RIGHT_1_ERROR,
  MOTOR_FRONT_RIGHT_2_ERROR,
  MOTOR_BACK_LEFT_1_ERROR,
  MOTOR_BACK_LEFT_2_ERROR,
  MOTOR_BACK_RIGHT_1_ERROR,
  MOTOR_BACK_RIGHT_2_ERROR,
  ENCODER_FRONT_LEFT_1_ERROR,
  ENCODER_FRONT_LEFT_2_ERROR,
  ENCODER_FRONT_RIGHT_1_ERROR,
  ENCODER_FRONT_RIGHT_2_ERROR,
  ENCODER_BACK_LEFT_1_ERROR,
  ENCODER_BACK_LEFT_2_ERROR,
  ENCODER_BACK_RIGHT_1_ERROR,
  ENCODER_BACK_RIGHT_2_ERROR,
};

/** Warning codes */
enum warnings
{
  NONE1,
  NONE2,
};

/** Operation states */
enum states
{
  S_IDLE,
  S_TRANSITION,
  S_STAND,
  S_LAYDOWN,
  S_CALIBRATE,
  S_TRANSITIONWALK,
  S_WALK,
  S_JUMP,
  S_AUTONOMOUS,
  S_MANUAL,
  S_RESET,
  S_INSPECT,

};

uint8_t nextState = NONE;
uint8_t currentState = S_IDLE;
uint8_t currentErrors[3] = {NONE};
uint8_t currentWarnings[3] = {NONE};
boolean transition = false;
uint8_t numberOfInspections = 0;

#include "types.h"
p autoParams;
p manualParams;

/**
 * Define the normal parameters
 */
#define normalSpeed PI / 140 //PI / 140
#define maxSpeed PI / 70     //PI / 70
#define normalStepLength 80  //80
#define normalHeight 170     //170
/**
 * Sets the standard values for the leg tracjetory
 */
void initializeLegTracjetory()
{
  autoParams.amplitude_over = 30.0; //30 fin sving
  autoParams.amplitude_under = 2.0; //10 fin sving 2 enda bedre
  autoParams.step_left = normalStepLength;
  autoParams.step_right = normalStepLength,
  autoParams.height = normalHeight;
  autoParams.frequency = 0;
  autoParams.period = 0;
  autoParams.x = 0;
  autoParams.dx = 0;

  manualParams.amplitude_over = 30.0;
  manualParams.amplitude_under = 2.0;
  manualParams.step_left = normalStepLength;
  manualParams.step_right = normalStepLength,
  manualParams.height = normalHeight;
  manualParams.frequency = 0;
  manualParams.period = 0;
  manualParams.x = 0;
  manualParams.dx = 0;
}

/* How fast the motors moves */
float robotVelocity = normalSpeed;
float pitchSetPoint = 0;

/*Makes it possible to trot*/
float PHASESHIFT0X = 3.14; //All legs togheter 3.14;    //Trot 3.14;
float PHASESHIFT0Y = 0;    //All legs togheter 0;       //Trot 0;

float PHASESHIFT1X = 3.14; //All legs togheter 0;       //Trot 3.14;
float PHASESHIFT1Y = 3.14; //All legs togheter 0;       //Trot 3.14;

float PHASESHIFT2X = 0;    //All legs togheter 3.14;    //Trot 0;
float PHASESHIFT2Y = 3.14; //All legs togheter 0;       //Trot 3.14;

float PHASESHIFT3X = 0; //All legs togheter 0;        //Trot 0;
float PHASESHIFT3Y = 0; //All legs togheter 0;        //Trot 0;

struct xboxControllerInputs
{
  float LJ_LEFT_RIGHT; // (-1.00 - 1.00) Default: 0.00
  float LJ_DOWN_UP;    // (-1.00 - 1.00) Default: 0.00
  float LT;            // (-1.00 - 1.00) Default: 1.00
  float RJ_LEFT_RIGHT; // (-1.00 - 1.00) Default: 0.00
  float RJ_DOWN_UP;    // (-1.00 - 1.00) Default: 0.00
  float RT;            // (-1.00 - 1.00) Default: 1.00
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

#endif // GLOBALS_H_
