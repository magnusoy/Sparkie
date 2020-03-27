#include "LegMovement.h"
#include "math.h"

/*



*/
LegMovement::LegMovement(void) {}

/**
 * returns the angle to put the motor in
 * @param x x value
 * @param y y value
 * @param motor motor 0 or 1
 * @param ODrive Odrive 0-3
*/
float LegMovement::compute(float x, float y, uint8_t motor, int ODrive)
{
  if ((ODrive == 0) || (ODrive == 2))
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
      if ((ODrive == 0) || (ODrive == 2))
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
float LegMovement::stepX(float n, float lenght, float frequency, float phaseShift)
{
  float x = lenght / 2 * sin(frequency * n + phaseShift);
  return x;
}

/**
 * Returns y
*/
float LegMovement::stepY(float n, float amplitudeOver, float amplitudeUnder, float robotHeight, float frequency, float phaseShift)
{
  float y;
  float wave = cos(frequency * n + phaseShift);
  if (wave > 0)
  {
    y = -robotHeight + amplitudeOver * wave;
  }
  else
  {
    y = -robotHeight + amplitudeUnder * wave;
  }
  return y;
}
