#ifndef _PID_H_
#define _PID_H_

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class PID
{

public:
//Defining Constants
#define DIRECT 0
#define REVERSE 1

	// Defining Functions
	PID(double kp, double ki, double kd, int direction);
	double compute(double actual, double target);
	void setParams(double kp, double ki, double kd);
	void setUpdateTime(unsigned long updateTime);
	void setDirection(int direction);
	void setOutputOffset(double offset);
	void setOutputLimits(double low, double high);
	void reset(void);

private:
	// Defining varibales
	double kp;
	double ki;
	double kd;
	double outputValue;
	double outputSum;
	double offset;
	double error;
	double outputLimitLow;
	double outputLimitHigh;
	unsigned long updateTime;
	unsigned long lastUpdateTime;
	int direction;
};

#endif // _PID_H_
