#include "LegMovement.h"
#include "math.h"
#include <Arduino.h>

LegMovement::LegMovement(ODriveArduino &_odrive, int _leg_number, float _phase_shift_x, float _phase_shift_y)
    : odrive(_odrive), leg_number(_leg_number), phase_shift_x(_phase_shift_x), phase_shift_y(_phase_shift_y)
{
  height = 170; // Not set
}

/**
 * returns the angle to put the motor in
 * @param x x value
 * @param y y value
 * @param motor motor 0 or 1
 * @param ODrive Odrive 0-3
*/
float LegMovement::compute(float x, float y, uint8_t motor)
{
  if ((leg_number == 0) || (leg_number == 2))
  {
    INNER = 0;
    OUTER = 1;
  }
  else
  {
    INNER = 1;
    OUTER = 0;
  }

  float alpha = 0;
  float r = sqrt((x * x) + (y * y));
  r = constrain(r, 80, 249);
  float theta = atan(x / y);
  float gamma = acos((8100 + (r * r) - 25600) / (180 * r));

  if (y < 0)
  {
    if (motor == INNER)
    {
      alpha = -gamma - theta;
    }
    else if (motor == OUTER)
    {
      alpha = gamma - theta;
    }
  }
  else if (y >= 0)
  {
    if (x == 0)
    {
      float offsetInRad;
      if ((leg_number == 0) || (leg_number == 2))
      {
        offsetInRad = -3.14;
      }
      else
      {
        offsetInRad = 3.14;
      }

      if (motor == INNER)
      {
        alpha = -gamma + offsetInRad;
      }
      else if (motor == OUTER)
      {
        alpha = gamma + offsetInRad;
      }
    }
    else if (x > 0)
    {
      if (motor == INNER)
      {
        alpha = -gamma + 1.57 + (1.57 - theta);
      }
      else if (motor == OUTER)
      {
        alpha = gamma + 1.57 + (1.57 - theta);
      }
    }
    else if (x < 0)
    {
      if (motor == INNER)
      {
        alpha = -gamma - 1.57 - (1.57 + theta);
      }
      else if (motor == OUTER)
      {
        alpha = gamma - 1.57 - (1.57 + theta);
      }
    }
  }
  alpha = alpha * 57.296;
  return alpha;
}

/**
 * Returns x
*/
float LegMovement::stepX(p &params, float phase_shift)
{
  if (leg_number == 0 || leg_number == 2)
  {
    this->x = params.step_right / 2 * sin(params.frequency * params.x + phase_shift);
  }
  else
  {
    this->x = params.step_left / 2 * sin(params.frequency * params.x + phase_shift);
  }

  return this->x;
}

/**
 * Returns y
*/
float LegMovement::stepY(p &params, float phase_shift)
{

  float wave = cos(params.frequency * params.x + phase_shift);
  if (wave > 0)
  {
    this->y = -this->height + params.amplitude_over * wave;
  }
  else
  {
    this->y = -this->height + params.amplitude_under * wave;
  }
  return this->y;
}

void LegMovement::linearMove(float x, float y)
{
  for (int motor = 0; motor < 2; motor++)
  {
    double angle = this->compute(x, y, motor);
    double motor_count = map(angle, -360, 360, -6000, 6000);
    this->odrive.TrapezoidalMove(motor, motor_count);
  }
}

void LegMovement::holdPosition(float x, float y)
{
  for (int motor = 0; motor < 2; motor++)
  {
    double angle = this->compute(x, y, motor);
    double motor_count = map(angle, -360, 360, -6000, 6000);
    this->odrive.TrapezoidalMove(motor, motor_count);
  }
}

void LegMovement::move(p &params)
{
  double x = this->stepX(params, phase_shift_x);
  double y = this->stepY(params, phase_shift_y);
  for (int motor = 0; motor < 2; motor++)
  {
    double angle = this->compute(x, y, motor);
    double motor_count = map(angle, -360, 360, -6000, 6000);
    this->odrive.TrapezoidalMove(motor, motor_count);
  }
}

void LegMovement::moveToGround(p &params)
{
  float y = params.height;
  holdPosition(this->x, -y);
}

void LegMovement::setHeight(float height)
{
  this->height = height;
}

void LegMovement::setPID(float P, float I, float D)
{
  for (int motor_number = 0; motor_number < 2; motor_number++)
  {
    odrive.writePID(motor_number, P, I, D);
  }
}

void LegMovement::setTrapTraj(float vel_limit, float accel_limit, float decel_limit)
{
  for (int motor_number = 0; motor_number < 2; motor_number++)
  {
    odrive.writeTrapTraj(motor_number, vel_limit, accel_limit, decel_limit);
  }
}