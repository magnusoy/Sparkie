#ifndef _LEGMOVEMENT_H_
#define _LEGMOVEMENT_H_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "types.h"
#include "ODriveArduino.h"

class LegMovement
{

public:
	int INNER;
	int OUTER;
	float x, y;

	LegMovement(ODriveArduino &_odrive, int _leg_number);
	float compute(float x, float y, uint8_t motor);
	float stepX(p &params, float phaseShift);
	float stepY(p &params, float phaseShift);
	void linearMove(float x, float y, int velocity);
	void move(p &params, float phase_shift);
	float getX();
	float getY();

private:
	ODriveArduino &odrive;
	int leg_number;
};

#endif // _LEGMOVEMENT_H_
