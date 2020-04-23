#ifndef LOCOMOTION_H_
#define LOCOMOTION_H_

#include "../lib/Timer/src/Timer.h"
#include "../lib/PID/src/PID.h"
#include "LegMovement.h"
#include "Globals.h"
#include "OdriveParameters.h"
#include "types.h"
#include "Navigation.h"

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

PID pitchPID(4, 0.001, 0, DIRECT); //I = 0.001
PID rollPID(2, 0.001, 0, REVERSE);
PID yawPID(1, 0, 0, REVERSE);
double pitchOutput = 0;
double rollOutput = 0;
double yawOutput;

/**
 * Sets the parameters for the PIDs
 */
void initializePIDs()
{
    pitchPID.setUpdateTime(5);
    pitchPID.setOutputLimits(-50, 50);
    rollPID.setUpdateTime(5);
    rollPID.setOutputLimits(-50, 50);
    //yawPID.setUpdateTime(5);
    //yawPID.setOutputLimits(-160, 160);
}

/**
 * Compute the PID outputs for making the robot level
 */
void computePIDs()
{
    pitchOutput = pitchPID.compute(ORIENTAION.pitch, pitchSetPoint);
    rollOutput = rollPID.compute(ORIENTAION.roll, 0);
    //yawOutput = yawOutput.compute(ORIENTAION.yaw,)
}

/**
 * Sets the height of each foot for making the robot level
 */
void computeHeight(p &params)
{
    computePIDs();
    Legs[0].setHeight(params, params.height - pitchOutput + rollOutput);
    Legs[1].setHeight(params, params.height - pitchOutput - rollOutput);
    Legs[2].setHeight(params, params.height + pitchOutput + rollOutput);
    Legs[3].setHeight(params, params.height + pitchOutput - rollOutput);
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
 * Set the PID parameters for the motors
 */
void setLegMotorPID(float P, float I, float D)
{
    for (uint8_t i = 0; i < 4; i++)
    { // Adjust PID gain on all legs
        Legs[i].setPID(P, I, D);
    }
}

/**
 * Set the trapezoidal trajectory limits
 */
void setLegMotorTrapTraj(float vel_limit, float accel_limit, float decel_limit)
{
    for (uint8_t i = 0; i < 4; i++)
    { // Adjust trap settings on all legs
        Legs[i].setTrapTraj(vel_limit, accel_limit, decel_limit);
    }
}

/**
 * Slowly moves the legs of the robot to a point
 * @param x, x value of the point
 * @param y, y value of the point
 */
void transitionToPoint(float x, float y)
{
    Legs[0].linearMove(-x, y);
    Legs[1].linearMove(x, y);
    Legs[2].linearMove(x, y);
    Legs[3].linearMove(-x, y);
}

/**
 Makes the robot stand in idle position and 
 holds the robot stable with the help of the
 IMU
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

void turningVelocity()
{

    if (robotVelocity >= 0 && robotVelocity < 0.2)
    {
        robotVelocity = normalSpeed;
    }
    else if (robotVelocity < 0 && robotVelocity > -0.2)
    {
        robotVelocity = -normalSpeed;
    }
}

void movementVelocity()
{
    if (NAVIGATION.VEL_LINEAR_X < 0.2 && NAVIGATION.VEL_LINEAR_X > -0.2)
    {
        robotVelocity = map(NAVIGATION.VEL_LINEAR_X, -1, 1, -normalSpeed, normalSpeed);
    }
    else if (NAVIGATION.VEL_LINEAR_X >= 0.2)
    {
        robotVelocity = normalSpeed;
    }
    else if (NAVIGATION.VEL_LINEAR_X <= -0.2)
    {
        robotVelocity = -normalSpeed;
    }
}

void mapNavigation()
{
    movementVelocity();
    if (NAVIGATION.VEL_ANGULAR_Z > 0)
    {
        turningVelocity();
        autoParams.step_left = normalStepLength;
        autoParams.step_right = map(NAVIGATION.VEL_ANGULAR_Z, 0, 1, normalStepLength, 0);
    }
    else if (NAVIGATION.VEL_ANGULAR_Z < 0)
    {
        turningVelocity();
        autoParams.step_left = map(NAVIGATION.VEL_ANGULAR_Z, -1, 0, 0, normalStepLength);
        autoParams.step_right = normalStepLength;
    }
    else
    {
        autoParams.step_left = normalStepLength;
        autoParams.step_right = normalStepLength;
    }
}

/**
 * Makes the robot walk,run,forwards,backwards etc
*/
void locomotion(p &params)
{
    if (robotVelocity == 0)
    {
        //Legs[0].moveToGround(params.height - pitchOutput + rollOutput);
        //Legs[1].moveToGround(params.height - pitchOutput - rollOutput);
        //Legs[2].moveToGround(params.height + pitchOutput + rollOutput);
        //Legs[3].moveToGround(params.height + pitchOutput - rollOutput);
        Legs[0].moveToGround(params.height);
        Legs[1].moveToGround(params.height);
        Legs[2].moveToGround(params.height);
        Legs[3].moveToGround(params.height);
    }
    else
    {
        Legs[0].move(params);
        Legs[1].move(params);
        Legs[2].move(params);
        Legs[3].move(params);
        shift(robotVelocity, params);
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
 * Makes the robot tilt for inspecting objects that is higher
 */
void inspect()
{
    Legs[0].linearMove(20, -200);
    Legs[1].linearMove(20, -200);
    Legs[2].linearMove(0, -80);
    Legs[3].linearMove(0, -80);
}

/**
 Makes the robot lay down on the ground
 */
void layDown()
{
    transitionToPoint(80, -5);
}

/**
 Makes the robot stand up from laying on the floor
 */
void standUp()
{
    transitionToPoint(70, -120);
}

/**
 Makes the robot start turning right
 */
void turnRight()
{
    autoParams.step_left = normalStepLength;
    autoParams.step_right = 0;
    locomotion(autoParams);
}

/**
 Makes the robot start turning left
 */
void turnLeft()
{
    autoParams.step_left = 0;
    autoParams.step_right = normalStepLength;
    locomotion(autoParams);
}

#endif // LOCOMOTION_H_