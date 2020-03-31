#ifndef ODRIVEPARAMETERS_H_
#define ODRIVEPARAMETERS_H_

/**
   Template for writing to  oDrives
*/
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
/**
   Template for writing to  oDrives
*/
template <>
inline Print &operator<<(Print &obj, float arg)
{
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

/** Odrive limitations */
#define MOTOR_SPEED_LIMIT 40000.0f
#define MOTOR_CURRENT_LIMIT 40.0f

/** Speed increase multiplier */
#define MOTOR_SPEED_MULTIPLIER 30
#define MOTOR_SPEED_LOWER 0
#define MOTOR_SPEED_UPPER 2000

/** Storing the positions of the motors */
int motorpositions[4][2] = {{0, 0},
                            {0, 0},
                            {0, 0},
                            {0, 0}};

/** Storing the current of the motors */
int motorcurrent[4][2] = {{0, 0},
                          {0, 0},
                          {0, 0},
                          {0, 0}};

/**
  Initialize the four Odrives.
*/
void initializeOdrives()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    hwSerials[i].begin(BAUDRATE);
  }
}

/**
   Set the given motor to the assigned position.
   @param odriveNumber, Specify oDrive (0-4)
   @param motorNumber, Specify motor {0, 1}
   @param pos, Position to drive to
*/
void setMotorPosition(const uint8_t odriveNumber, const uint8_t motorNumber, double pos)
{
  odrives[odriveNumber].SetPosition1(motorNumber, pos, 50000.000);
}

void linearMove(const uint8_t odriveNumber, const uint8_t motorNumber, double pos)
{
  odrives[odriveNumber].SetPosition1(motorNumber, pos, 5000.000);
}
/**
  Sets the motors in desired state
*/
void setOdrivesInState(uint8_t requestedState, uint8_t wait)
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    for (uint8_t m = 0; m < 2; ++m)
    {
      odrives[i].run_state(m, requestedState, wait);
      delay(10);
    }
  }
}

/**
  Calibreates motors.
  Be aware the motors will move during this process!
*/
void calibrateOdriveMotors()
{
  uint8_t requestedState = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  setOdrivesInState(requestedState, true);
  requestedState = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  setOdrivesInState(requestedState, true);
}

/*
 * Activate the motors
*/
void armMotors()
{
  uint8_t requestedState = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  setOdrivesInState(requestedState, false);
}
/*
 * Deactivate the motors
*/
void disarmMotors()
{
  uint8_t requestedState = ODriveArduino::AXIS_STATE_IDLE;
  setOdrivesInState(requestedState, false);
}

/**
  Read current motor position from Odrive.
  Storing them in the global motorPosition
  variable.
*/
void readOdriveMotorPositions(HardwareSerial hwSerials[], ODriveArduino odrives[])
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    for (uint8_t m = 0; m < 2; ++m)
    {
      hwSerials[i] << "r axis" << m << ".encoder.pos_estimate\n";
      motorpositions[i][m] = odrives[i].readFloat();
    }
  }
}

/**
  Read motor current from Odrive.
  Storing them in the global motorCurrent
  variable.
*/
void readOdriveMotorCurrent()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    for (uint8_t m = 0; m < 2; ++m)
    {
      motorcurrent[i][m] = odrives[i].GetCurrent(m);
    }
  }
}

/**
   Print all errors on the motors
*/
void checkForErrors()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    for (uint8_t axis = 0; axis < 2; ++axis)
    {
      odrives[i].checkForErrors(axis);
    }
  }
}
/**
   Resets all errors on all motors
*/
void resetMotorsErrors()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    for (uint8_t axis = 0; axis < 2; ++axis)
    {
      odrives[i].resetErrors(axis);
    }
  }
}

/**
   Read all configs
*/
void readConfig()
{
  Serial << "Reading ODrive configuration... \n";
  for (uint8_t i = 0; i < 4; ++i)
  {
    Serial << "Odrive number: " << i << "\n";
    for (uint8_t axis = 0; axis < 2; ++axis)
    {
      odrives[i].readConfig(axis);
    }
  }
}

/**
 * Sets the precalibrated options on the motor to true or false
 * @param var true or false
 */
void setPreCalibrated(bool var)
{
  Serial << "Pre Calibrate... \n";
  for (uint8_t i = 0; i < 4; ++i)
  {
    if (2 != i)
    {
      Serial << "Odrive number: " << i << "\n";
      for (uint8_t axis = 0; axis < 2; ++axis)
      {
        odrives[i].setPreCalibrated(axis, var);
      }
    }
  }
}
/**
 * Saves the Odrive config
 */
void saveConfigOdrives()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    odrives[i].saveConfig();
    delay(200);
  }
}

/**
 * Reboot the Odrives
 */
void rebootOdrives()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    odrives[i].reboot();
    delay(200);
  }
}
#endif // ODRIVEPARAMETERS_H_
