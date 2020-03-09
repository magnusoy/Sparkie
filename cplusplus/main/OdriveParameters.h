#ifndef _ODRIVEPARAMETERS_H_
#define _ODRIVEPARAMETERS_H_

/**
   Template for writing to  oDrives
*/
template<class T> inline Print& operator <<(Print & obj,     T arg) {
  obj.print(arg);
  return obj;
}
/**
   Template for writing to  oDrives
*/
template<>        inline Print& operator <<(Print & obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

/*Serial connection for each odrive*/
HardwareSerial hwSerials[4] = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

/*Odrive motordrivers, one Odrive per foot*/
ODriveArduino odriveFrontLeft(FRONT_LEFT);
ODriveArduino odriveFrontRight(FRONT_RIGHT);
ODriveArduino odriveBackLeft(BACK_LEFT);
ODriveArduino odriveBackRight(BACK_RIGHT);

ODriveArduino odrives[4] = {odriveFrontLeft, odriveFrontRight, odriveBackLeft, odriveBackRight};

/**
  Initialize the four Odrives.
*/
void initializeOdrives() {
  for (int i = 0; i < 1; ++i) {
    hwSerials[i].begin(BAUDRATE);
  }
}

/**
   Set the given motor to the assigned position.
   @param odriveNumber, Specify oDrive (0-4)
   @param motorNumber, Specify motor {0, 1}
   @param pos, Position to drive to
*/
void setMotorPosition(const int odriveNumber, const int motorNumber, double pos) {
  odrives[odriveNumber].SetPosition(motorNumber, pos);
}


/** Odrive limitations */
#define MOTOR_SPEED_LIMIT 40000.0f
#define MOTOR_CURRENT_LIMIT 40.0f

/** Speed increase multiplier */
#define MOTOR_SPEED_MULTIPLIER 30
#define MOTOR_SPEED_LOWER 0
#define MOTOR_SPEED_UPPER 2000

/** Motor structure */
//enum motors {
//  INNER,
//  OUTER,
//};

/** Storing the positions of the motors */
int motorpositions[4][2] = {{0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};

/**
  Sets the motors in desired state
*/
void setOdrivesInState(ODriveArduino odrives[], uint8_t requestedState, uint8_t wait) {
  for (int i = 0; i < 1; ++i) {
    for (int m = 0; m < 2; ++m) {
      odrives[i].run_state(m, requestedState, wait);
    }
  }
}

/**
  Calibreates motors.
  Be aware the motors will move during this process!
*/
void calibrateOdriveMotors(ODriveArduino odrives[0]) {
  uint8_t requestedState = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  setOdrivesInState(odrives, requestedState, true);
  requestedState = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  setOdrivesInState(odrives, requestedState, true);

}

void armMotors(ODriveArduino odrives[0]) {
  uint8_t requestedState = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  setOdrivesInState(odrives, requestedState, false);
}
void disarmMotors(ODriveArduino odrives[0]) {
  uint8_t requestedState = ODriveArduino::AXIS_STATE_IDLE;
  setOdrivesInState(odrives, requestedState, false);
}

/**
  Read current motor position from Odrive.
  Storing them in the global motorPosition
  variable.
*/
void readOdriveMotorPositions(HardwareSerial hwSerials[] , ODriveArduino odrives[]) {
  for (int i = 0; i < 5; ++i) {
    for (int m = 0; m < 2; ++m) {
      hwSerials[i] << "r axis" << m << ".encoder.pos_estimate\n";
      motorpositions[i][m] = odrives[i].readFloat();
    }
  }
}

#endif // _ODRIVEPARAMETERS_H_
