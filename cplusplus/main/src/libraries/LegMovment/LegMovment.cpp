#include "LegMovment.h"
#include "math.h"

/*



*/
LegMovment::LegMovment(void){}


/*


*/
double LegMovment::compute(double x, double y, uint8_t motor)
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
double LegMovment::stepX(unsigned long n, double lenght, double frequency)
{
	double x = lenght / 2 * sin(6.28318530718 * frequency * n);
	return x;
}

/*


*/
double LegMovment::stepY(unsigned long n, double amplitude, double height, double frequency)
{
	double y = -height + amplitude * cos(6.28318530718 * frequency * n);
	return y;
}
