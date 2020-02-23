#include "InverseKinematics.h"
#include "math.h"

/*



*/
InverseKinematics::InverseKinematics(void){}


/*


*/
double InverseKinematics::compute(double x, double y, uint8_t motor)
{
	double alpha;
	double r = sqrt((x * x) + (y * y));
	double theta = atan(y / x);
	double gamma = acos((8100 + (r * r) - 25600) / (180 * r));
	//double gammaInDegrees = gamma * 57.296;

	if (x < 0)
	{
		theta = theta - 3.14159265359;
	}
	if (motor == INNER)
	{
		alpha = gamma - theta;
	}
	else if (motor == OUTER)
	{
		alpha = gamma + theta;
	}
	alpha = alpha * 57.296;

	return alpha;
}

/*


*/
double InverseKinematics::stepX(unsigned long n, double lenght, double frequency)
{
	double x = lenght / 2 * sin(6.28318530718 * frequency * n);
	return x;
}

/*


*/
double InverseKinematics::stepY(unsigned long n, double amplitude, double height, double frequency)
{
	double y = -height + amplitude * cos(6.28318530718 * frequency * n);
	return y;
}
