#include "LegMovement.h"
#include "math.h"

/*



*/
LegMovement::LegMovement(void) {}

/*


*/
double LegMovement::compute(double x, double y, uint8_t motor, int ODrive)
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

  double alpha = 0;
  double r = sqrt((x * x) + (y * y));
  r = constrain(r, 80, 249);
  double theta = atan(x / y);
  double gamma = acos((8100 + (r * r) - 25600) / (180 * r));

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
      double offsetInRad;
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

/*


*/
double LegMovement::stepX(unsigned long n, double lenght, double frequency, double phaseShift)
{

  double x = lenght / 2 * sin(frequency * n + phaseShift);
  return x;
}

/*


*/
double LegMovement::stepY(unsigned long n, double amplitudeOver, double amplitudeUnder, double robotHeight, double frequency, double phaseShift)
{
  double y;
  double wave = cos(frequency * n + phaseShift);
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
