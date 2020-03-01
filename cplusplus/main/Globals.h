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

/* Parameters for leg movment*/
double AMPLITUDE = 20.0f;
double LENGHT = 30.0f;
double HEIGHT = 200.0f;
double FREQUENCY = 2.0f;
#endif // _GLOBALS_H_
