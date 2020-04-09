#include "PID.h"
#include "math.h"

/*
	PID constructor, initializing controller terms
	and direction.

	@param kp - proportional gain factor
	@param ki - integral gain factor
	@param kd  derivative gain factor
	@param direction - DIRECT or REVERSE
*/
PID::PID(double kp, double ki, double kd, int direction)
{
	this->direction = direction;
	PID::setParams(kp, ki, kd);
	lastUpdateTime = millis();
}

/*
	Computes the new output based on controller terms
	and error.

	@param actual - current value
	@param target - wanted value

	@return outputValue - calculated output
*/
double PID::compute(double actual, double target)
{
	if (lastUpdateTime + updateTime <= millis())
	{
		double lastError = error;
		error = target - actual;
		double output = error * kp;
		outputSum += error * ki;
		double outputLast = (lastError - error) * kd;
		outputValue = offset + output + outputSum + outputLast;
		if (outputValue > outputLimitHigh)
		{
			outputValue = outputLimitHigh;
		}
		if (outputValue < outputLimitLow)
		{
			outputValue = outputLimitLow;
		}
		lastUpdateTime = millis();
	}
	return outputValue;
}

/*
	Resets the global variables.
*/
void PID::reset(void)
{
	//bool reset = false;
	lastUpdateTime = millis();
	error = 0.0f;
	outputSum = 0.0f;
	outputValue = 0.0f;
}

/*
	Set the constrain limits for the output.

	@param low - the lowest output
	@param high - the highest output
*/
void PID::setOutputLimits(double low, double high)
{
	this->outputLimitLow = low;
	this->outputLimitHigh = high;
}

/*
	Adds a offset on top of the output.

	@param offset - the offset value to be added
*/
void PID::setOutputOffset(double offset) { this->offset = offset; }

/*
	Set the how often the output should update.

	@param updateTime - update time in millis
*/
void PID::setUpdateTime(unsigned long updateTime) { this->updateTime = updateTime; }

/*
	Set the controller terms.

	@param kp - proportional gain factor
	@param ki - integral gain factor
	@param kd  derivative gain factor
*/
void PID::setParams(double kp, double ki, double kd)
{
	if (direction == 0)
	{
		this->kp = kp;
		this->ki = ki;
		this->kd = kd;
	}
	else if (direction == 1)
	{
		this->kp = -kp;
		this->ki = -ki;
		this->kd = -kd;
	}
}
