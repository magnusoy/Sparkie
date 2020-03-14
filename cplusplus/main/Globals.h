#ifndef _GLOBALS_H_
#define _GLOBALS_H_
#include "Constants.h"

/** IMU structure */
typedef struct {
  float yaw;
  float pitch;
  float roll;
} IMU;

/** Accelerometer structure */
typedef struct {
  float x;
  float y;
  float z;
} Accelerometer;

/** Gyroscope structure */
typedef struct {
  float x;
  float y;
  float z;
} Gyroscope;

/** Error codes */
enum errors {
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
enum warnings {
  NONE1,
  NONE2,
};

/** Operation states */
enum states {
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
uint8_t calibrated = false;

/* Parameters for leg movment*/
double AMPLITUDEOVER = 70.0;//NORMAL 70.0;
double AMPLITUDEUNDER = 30.0; //NORMAL 30.0;
double LENGHT = 160.0; //NORMAL 160.0;
double HEIGHT = 170.0; //NORMAL 170.0;
double FREQUENCY = 0.025; //SLOW 0.0025 //WALK 0.025  //RUN 0.05

double PHASESHIFT0X = 3.14;    //Alle ben samlet 3.14;    //Trot 3.14;
double PHASESHIFT0Y = 0;       //Alle ben samlet 0;       //Trot 0;

double PHASESHIFT1X = 3.14;       //Alle ben samlet 0;       //Trot 3.14;
double PHASESHIFT1Y = 3.14;       //Alle ben samlet 0;       //Trot 3.14;

double PHASESHIFT2X = 0;    //Alle ben samlet 3.14;    //Trot 0;  
double PHASESHIFT2Y = 3.14;       //Alle ben samlet 0;       //Trot 3.14;

double PHASESHIFT3X = 0;      //Alle ben samlet 0;        //Trot 0;
double PHASESHIFT3Y = 0;      //Alle ben samlet 0;        //Trot 0;
#endif // _GLOBALS_H_
