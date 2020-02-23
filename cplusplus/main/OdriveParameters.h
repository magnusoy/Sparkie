#ifndef _ODRIVEPARAMETERS_H_
#define _ODRIVEPARAMETERS_H_

template<class T> inline Print& operator <<(Print & obj,     T arg) {
  obj.print(arg);
  return obj;
}

template<>        inline Print& operator <<(Print & obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

/** Odrive limitations */
#define MOTOR_SPEED_LIMIT 40000.0f
#define MOTOR_CURRENT_LIMIT 40.0f

/** Speed increase multiplier */
#define MOTOR_SPEED_MULTIPLIER 30
#define MOTOR_SPEED_LOWER 0
#define MOTOR_SPEED_UPPER 2000

/** Motor structure */
enum motors {
  INNER,
  OUTER,
};

int motorpositions[4][2] = {{0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};


/**
 
*/
void setOdrivesInState(ODriveArduino odrives[], uint8_t requestedState, uint8_t wait) {
  for (int i = 0; i < 1; ++i) {
    for (int m = 0; m < 2; ++m) {
      odrives[i].run_state(m, requestedState, wait);
    }
  }
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

/**
 
*/
void calibrateOdriveMotors(ODriveArduino odrives[]) {
  uint8_t requestedState = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  setOdrivesInState(odrives, requestedState, true);
  requestedState = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  setOdrivesInState(odrives, requestedState, true);
  requestedState = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  setOdrivesInState(odrives, requestedState, false);
}

#endif // _ODRIVEPARAMETERS_H_
