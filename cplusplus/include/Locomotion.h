#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include "../lib/Timer/src/Timer.h"
#include "../lib/PID/src/PID.h"
#include "LegMovement.h"
#include "Globals.h"
#include "OdriveParameters.h"
#include "types.h"

LegMovement legMovement0(odrives[0], 0, PHASESHIFT0X, PHASESHIFT0Y);
LegMovement legMovement1(odrives[1], 1, PHASESHIFT1X, PHASESHIFT1Y);
LegMovement legMovement2(odrives[2], 2, PHASESHIFT2X, PHASESHIFT2Y);
LegMovement legMovement3(odrives[3], 3, PHASESHIFT3X, PHASESHIFT3Y);

LegMovement Legs[4] = {legMovement0, legMovement1, legMovement2, legMovement3};

/* Variable for jump fuction*/
Timer airTime;
Timer groundTime;
bool runned = false;
uint8_t jump = 0;

/* Variable for lay down*/
float x = 0;

PID pitchPID(12, 0.0001, 0, DIRECT); //I = 0.001
PID rollPID(8, 0.0001, 0, REVERSE);
PID yawPID(1, 0, 0, REVERSE);
double pitchOutput;
double rollOutput;
double yawOutput;

void initializePIDs()
{
    pitchPID.setUpdateTime(5);
    pitchPID.setOutputLimits(-50, 50);
    rollPID.setUpdateTime(5);
    rollPID.setOutputLimits(-50, 50);
    //yawPID.setUpdateTime(5);
    //yawPID.setOutputLimits(-160, 160);
}

void computePIDs()
{
    pitchOutput = pitchPID.compute(ORIENTAION.pitch, 0);
    rollOutput = rollPID.compute(ORIENTAION.roll, 0);
    //yawOutput = yawOutput.compute(ORIENTAION.yaw,)
}

void computeAutoParams()
{
    legMovement0.setHeight(autoParams.height + pitchOutput - rollOutput);
    legMovement1.setHeight(autoParams.height + pitchOutput + rollOutput);
    legMovement2.setHeight(autoParams.height - pitchOutput - rollOutput);
    legMovement3.setHeight(autoParams.height - pitchOutput + rollOutput);
}

void computeManualParams()
{
    legMovement0.setHeight(manualParams.height + pitchOutput - rollOutput);
    legMovement1.setHeight(manualParams.height + pitchOutput + rollOutput);
    legMovement2.setHeight(manualParams.height - pitchOutput - rollOutput);
    legMovement3.setHeight(manualParams.height - pitchOutput + rollOutput);
}

/**
 * 
*/
void mod_constrain(float &val, float period)
{
    if (val * 2 * PI > period || val < 0)
    {
        val = fmodf(val, period);
    }
}

/**
 * Shifts the x value in positive or negative direction
*/
void shift(float dx, p &par)
{
    par.dx = dx;
    par.x += dx; // add increment or decrement
    mod_constrain(par.x, par.period);
}

/**
 * Sets the frequency to your choosing
*/
void set_frequency(float freq, p &par)
{
    par.period = (1 / freq) * 2 * PI; // calculate period
    par.frequency = freq;             // set frequency
}

/**
 TODO: Add docstring
 */
void setLegMotorPID(float P, float I, float D)
{
    for (uint8_t i = 0; i < 4; i++)
    { // Adjust PID gain on all legs
        Legs[i].setPID(P, I, D);
    }
}

/**
 TODO: Add docstring
 */
void setLegMotorTrapTraj(float vel_limit, float accel_limit, float decel_limit)
{
    for (uint8_t i = 0; i < 4; i++)
    { // Adjust trap settings on all legs
        Legs[i].setTrapTraj(vel_limit, accel_limit, decel_limit);
    }
}

/**
 TODO: Add docstring
 */
void transitionToPoint(float x, float y)
{
    Legs[0].linearMove(-x, y);
    Legs[1].linearMove(x, y);
    Legs[2].linearMove(x, y);
    Legs[3].linearMove(-x, y);
}

/**
 TODO: Add docstring
 */
void stand()
{
    float x = 70;
    float y = -120;
    Legs[0].holdPosition(-x, y + pitchOutput - rollOutput);
    Legs[1].holdPosition(x, y + pitchOutput + rollOutput);
    Legs[2].holdPosition(x, y - pitchOutput - rollOutput);
    Legs[3].holdPosition(-x, y - pitchOutput + rollOutput);
}

/**
 * Sets the motors in idle position
*/
void setIdlePosition()
{
    transitionToPoint(70, -120);
}

/**
 * Makes the robot walk,run,forwards,backwards etc
*/
void locomotion(p &params)
{
    if (val == 0)
    {
        Legs[0].moveToGround(params);
        Legs[1].moveToGround(params);
        Legs[2].moveToGround(params);
        Legs[3].moveToGround(params);
    }
    else
    {
        Legs[0].move(params);
        Legs[1].move(params);
        Legs[2].move(params);
        Legs[3].move(params);
        shift(val, params);
    }
}

/**
 * Making the robot jump
*/
void jumpCommand()
{
    double x = 0;
    double y;
    switch (jump)
    {
    case 0:
        y = -80;

        if (!runned)
        {
            for (uint8_t Odrive = 0; Odrive < 4; Odrive++)
            {
                Legs[Odrive].linearMove(x, y);
            }
            groundTime.startTimer(500);
            runned = true;
        }
        if (groundTime.hasTimerExpired())
        {
            jump = 1;
            runned = false;
        }
        break;

    case 1:
        y = -200;
        if (!runned)
        {
            for (uint8_t Odrive = 0; Odrive < 4; Odrive++)
            {
                Legs[Odrive].linearMove(x, y);
            }
            airTime.startTimer(1000);
            runned = true;
        }
        if (airTime.hasTimerExpired())
        {
            jump = 0;
            runned = false;
        }
        break;
    }
}

/**
 TODO: Add docstring
 */
void layDown()
{
    transitionToPoint(80, -5);
}

/**
 TODO: Add docstring
 */
void standUp()
{
    transitionToPoint(70, -120);
}

/**
 TODO: Add docstring
 */
void turnLeft()
{
    autoParams.step_left = 0;
    autoParams.step_right = 160;
    locomotion(autoParams);
}

/**
 TODO: Add docstring
 */
void turnRight()
{
    autoParams.step_left = 160;
    autoParams.step_right = 0;
    locomotion(autoParams);
}

#endif // LOCOMOTION_H_