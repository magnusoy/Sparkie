#ifndef _LEGMOVEMENT_H_
#define _LEGMOVEMENT_H_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class LegMovement 
{

public:
int INNER;
int OUTER;

	LegMovement(void);
	double compute(double x, double y, uint8_t motor, int ODrive);
	double stepX(unsigned long n, double lenght, double frequency);
	double stepY(unsigned long n, double amplitudeOver,double amplitudeUnder, double robotHeight, double frequency);

private:
	bool step_direction = false; // Forward = false, backward = true

};

#endif // _LEGMOVEMENT_H_
