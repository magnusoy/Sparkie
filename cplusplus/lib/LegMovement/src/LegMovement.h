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
	float compute(float x, float y, uint8_t motor, int ODrive);
	float stepX(float n, float lenght, float frequency, float phaseShift);
	float stepY(float n, float amplitudeOver, float amplitudeUnder, float robotHeight, float frequency, float phaseShift);

private:
};

#endif // _LEGMOVEMENT_H_
