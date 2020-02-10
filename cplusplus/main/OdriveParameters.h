#ifndef _ODRIVEPARAMETERS_H_
#define _ODRIVEPARAMETERS_H_


/** Odrive limitations */
#define MOTOR_SPEED_LIMIT 40000.0f
#define MOTOR_CURRENT_LIMIT 40.0f

/** Speed increase multiplier */
#define MOTOR_SPEED_MULTIPLIER 30
#define MOTOR_SPEED_LOWER 0
#define MOTOR_SPEED_UPPER 2000

/**
  Template for printing
  to ODrive v3.6
*/
template<class T> inline Print& operator <<(Print & obj,     T arg) {
  obj.print(arg);
  return obj;
}

/**
  Template for printing
  to ODrive v3.6
*/
template<>        inline Print& operator <<(Print & obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

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
  Sets the motors in closed loop
  control mode.
*/
void setOdrivesInControlMode(ODriveArduino odrives[]) {
  int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  for (int i = 0; i < 5; ++i) {
    for (int m = 0; m < 2; ++m) {
      odrives[i].run_state(m, requested_state, false); // don't wait
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
  Configure motor parameters.
*/
void configureOdriveMotors(HardwareSerial hwSerials[] , ODriveArduino odrives[]) {
  for (int i = 0; i < 5; ++i) {
    for (int m = 0; m < 2; ++m) {
      hwSerials[i] << "w axis" << m << ".controller.config.vel_limit " << MOTOR_SPEED_LIMIT << '\n';
      hwSerials[i] << "w axis" << m << ".motor.config.current_lim " << MOTOR_CURRENT_LIMIT << '\n';
    }
  }
}

/**
  Stop motors immediately.
*/
void terminateOdriveMotors(ODriveArduino odrives[]) {
  int requested_state = requested_state = ODriveArduino::AXIS_STATE_IDLE;
  for (int i = 0; i < 5; ++i) {
    for (int m = 0; m < 2; ++m) {
      odrives[i].run_state(m, requested_state, true);
    }
  }
}

#endif // _ODRIVEPARAMETERS_H_
