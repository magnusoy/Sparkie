#ifndef _LEGMOVMENT_H_
#define _LEGMOVMENT_H_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class LegMovment 
{

public:
#define INNER 1
#define OUTER 0

	LegMovment(void);
	double compute(double x, double y, uint8_t motor);
	double stepX(unsigned long n, double lenght, double frequency);
	double stepY(unsigned long n, double amplitude, double height, double frequency);

};

#endif // _LEGMOVMENT_H_
