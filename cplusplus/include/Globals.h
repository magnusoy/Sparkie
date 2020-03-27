#ifndef GLOBALS_H_
#define GLOBALS_H_
#include "Constants.h"

/** IMU structure */
typedef struct
{
  float yaw;
  float pitch;
  float roll;
} IMU;

/** Accelerometer structure */
typedef struct
{
  float x;
  float y;
  float z;
} Accelerometer;

/** Gyroscope structure */
typedef struct
{
  float x;
  float y;
  float z;
} Gyroscope;

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
  S_CALIBRATE,
  S_READY,
  S_PAUSE,
  S_WALK,
  S_RUN,
  S_JUMP,
  S_AUTONOMOUS,
  S_MANUAL,
  S_BACKFLIP,
  S_CONFIGURE,
  S_RESET,
  S_WARNING,
  S_ERROR,

};

uint8_t currentState = S_IDLE;
uint8_t currentErrors[3] = {NONE};
uint8_t currentWarnings[3] = {NONE};
uint8_t idlePosition = false;
uint8_t calibrated = false; //Put false;

/* Variable for leg tracjetory */
typedef struct
{
  float amplitude_over = 70.0;  //NORMAL 70.0;
  float amplitude_under = 30.0; //NORMAL 30.0;
  float step_left = 160.0;      //NORMAL 160.0;
  float step_right = 160.0;     //NORMAL 160.0;
  float height = 170.0;         //NORMAL 170.0;
  float frequency = 0;          //NORMAL 0; is set in setup
  float period = 0;
  float x = 0;
  float dx = 0;
} p;

p *autoParams;
p *manualParams;

/**
 * Sets the standard values for the leg tracjetory
 */
void initializeLegTracjetory()
{
  autoParams->amplitude_over = 70.0;
  autoParams->amplitude_under = 30.0;
  autoParams->step_left = 160.0;
  autoParams->step_right = 160.0,
  autoParams->height = 170.0;
  autoParams->frequency = 0;
  autoParams->period = 0;
  autoParams->x = 0;
  autoParams->dx = 0;

  manualParams->amplitude_over = 70.0;
  manualParams->amplitude_under = 30.0;
  manualParams->step_left = 160.0;
  manualParams->step_right = 160.0,
  manualParams->height = 170.0;
  manualParams->frequency = 0;
  manualParams->period = 0;
  manualParams->x = 0;
  manualParams->dx = 0;
}

/* How fast the motors moves */
float val = PI / 70;

/*Makes it possible to trot*/
float PHASESHIFT0X = 3.14; //All legs togheter 3.14;    //Trot 3.14;
float PHASESHIFT0Y = 0;    //All legs togheter 0;       //Trot 0;

float PHASESHIFT1X = 3.14; //All legs togheter 0;       //Trot 3.14;
float PHASESHIFT1Y = 3.14; //All legs togheter 0;       //Trot 3.14;

float PHASESHIFT2X = 0;    //All legs togheter 3.14;    //Trot 0;
float PHASESHIFT2Y = 3.14; //All legs togheter 0;       //Trot 3.14;

float PHASESHIFT3X = 0; //All legs togheter 0;        //Trot 0;
float PHASESHIFT3Y = 0; //All legs togheter 0;        //Trot 0;
#endif                  // GLOBALS_H_
