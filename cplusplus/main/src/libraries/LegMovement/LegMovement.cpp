#include "LegMovement.h"
#include "math.h"

/*



*/
LegMovement::LegMovement(void){}

/*


*/
double LegMovement::compute(double x, double y, uint8_t motor)
  {
  double alpha;
  double r = sqrt((x * x) + (y * y));
  double theta = atan(x / y);
  double gamma = acos((8100 + (r * r) - 25600) / (180 * r));

  if (motor == INNER) {
    alpha = -gamma - theta;
  } else if (motor == OUTER) {
    alpha = gamma - theta;
  }
  alpha = alpha * 57.296;
  return alpha;
}

/*


*/
double LegMovement::stepX(unsigned long n, double lenght, double frequency)
{
  
	double x = lenght / 2 * sin(6.28 * frequency * n);
	return x;
}

/*


*/
double LegMovement::stepY(unsigned long n, double amplitudeOver,double amplitudeUnder, double robotHeight, double frequency)
{
	  double y;
  if (step_direction) {
    y = -robotHeight + amplitudeOver * cos(6.28 * frequency * n);
  } else {
    y = -robotHeight + amplitudeUnder * cos(6.28 * frequency * n);
  }
  step_direction = (robotHeight + y < 0) ? false : true;
  return y;
}
