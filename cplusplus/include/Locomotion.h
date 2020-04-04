#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include "../lib/Timer/src/Timer.h"
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

void setLegMotorPID(float P, float I, float D)
{
    for (uint8_t i = 0; i < 4; i++)
    { // Adjust PID gain on all legs
        Legs[i].setPID(P, I, D);
    }
}

void transitionToPoint(float x, float y)
{
    //setLegMotorPID(5.0f, 0.001f, 0.0f);
    float speed = 10000;
    Legs[0].linearMove(-x, y, speed);
    Legs[1].linearMove(x, y, speed);
    Legs[2].linearMove(x, y, speed);
    Legs[3].linearMove(-x, y, speed);
}

/**
 * Sets the motors in idle position
*/
void setIdlePosition()
{
    armMotors();
    delay(10);
    transitionToPoint(70, -120);
    //disarmMotors();
    idlePosition = true;
}

/**
 * Makes the robot walk,run,forwards,backwards etc
*/
void locomotion(p &params)
{
    Legs[0].move(params);
    Legs[1].move(params);
    Legs[2].move(params);
    Legs[3].move(params);
    shift(val, params);
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
                Legs[Odrive].linearMove(x, y, 5000.0);
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
                Legs[Odrive].linearMove(x, y, 50000.0);
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

void layDown()
{
    transitionToPoint(80, -5);
}

void standUp()
{
    transitionToPoint(0, -160);
}

void turnLeft()
{
    autoParams.step_left = 0;
    autoParams.step_right = 160;
    locomotion(autoParams);
}

void turnRight()
{
    autoParams.step_left = 160;
    autoParams.step_right = 0;
    locomotion(autoParams);
}

#endif // LOCOMOTION_H_