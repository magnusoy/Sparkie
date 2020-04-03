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

	LegMovement(ODriveArduino &_odrive, int _leg_number, float _phase_shift_x, float phase_shift_y);
	float compute(float x, float y, uint8_t motor);
	float stepX(p &params, float phaseShift);
	float stepY(p &params, float phaseShift);
	void linearMove(float x, float y, float velocity);
	void move(p &params);
	float getX();
	float getY();
	float setHeight(float height);
	void setPID(float P, float I, float D);

private:
	ODriveArduino &odrive;
	int leg_number;
	float phase_shift_x;
	float phase_shift_y;

	float height;
};

#endif // _LEGMOVEMENT_H_
