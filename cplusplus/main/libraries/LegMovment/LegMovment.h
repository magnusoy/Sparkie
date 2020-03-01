#ifndef _LegMovment_H_
#define _LegMovment_H_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class LegMovment
{

public:
#define INNER 0
#define OUTER 1

	LegMovment(void);
	double compute(double x, double y, uint8_t motor);
	double stepX(unsigned long n, double lenght, double frequency);
	double stepY(unsigned long n, double amplitude, double height, double frequency);

};

#endif // _LegMovment_H_
