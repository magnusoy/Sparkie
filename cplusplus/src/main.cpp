/**
  The purpose of this project ...
  Libraries used:
  ArduinoOdrive - https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino
  ArduinoJSON - https://github.com/bblanchon/ArduinoJson
  -----------------------------------------------------------
  Code by: Magnus Kvendseth Øye, Vegard Solheim, Petter Drønnen
  Date: 27.03-2020
  Version: 0.5
  Website: https://github.com/magnusoy/Sparkie
*/

// Including libraries and headers
#include <Arduino.h>
#include "../lib/ODriveArduino/src/ODriveArduino.h"
#include "../lib/Timer/src/Timer.h"
#include "../lib/ros_lib/ros.h"
#include "../lib/ros_lib/nav_msgs/Odometry.h"
#include "../lib/ros_lib/sensor_msgs/Joy.h"

#include "Globals.h"
#include "Constants.h"
#include "types.h"
#include "OdriveParameters.h"
#include "IO.h"
#include "XboxController.h"
#include "Locomotion.h"

/* ROS Nodehandlers */
ros::NodeHandle nh;

/**
 Joystick callback function for updating
 robot xbox controller inputs

 @param joy is the joystick msg
*/
void joyCallback(const sensor_msgs::Joy &joy)
{
  XBOX_CONTROLLER_INPUT.LJ_LEFT_RIGHT = joy.axes[0];
  XBOX_CONTROLLER_INPUT.LJ_DOWN_UP = joy.axes[1];
  XBOX_CONTROLLER_INPUT.LT = joy.axes[2];
  XBOX_CONTROLLER_INPUT.RJ_DOWN_UP = joy.axes[3];
  XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT = joy.axes[4];
  XBOX_CONTROLLER_INPUT.RT = joy.axes[5];
  XBOX_CONTROLLER_INPUT.A = joy.buttons[0];
  XBOX_CONTROLLER_INPUT.B = joy.buttons[1];
  XBOX_CONTROLLER_INPUT.X = joy.buttons[2];
  XBOX_CONTROLLER_INPUT.Y = joy.buttons[3];
  XBOX_CONTROLLER_INPUT.LB = joy.buttons[4];
  XBOX_CONTROLLER_INPUT.RB = joy.buttons[5];
  XBOX_CONTROLLER_INPUT.MLB = joy.buttons[6];
  XBOX_CONTROLLER_INPUT.MRB = joy.buttons[7];
  XBOX_CONTROLLER_INPUT.MB = joy.buttons[8];
  XBOX_CONTROLLER_INPUT.LJ = joy.buttons[9];
  XBOX_CONTROLLER_INPUT.RJ = joy.buttons[10];
}

/**
 Odometry callback function for updating
 robot position and orientation.

 @param odom is the odometry msg
*/
void odomCallback(const nav_msgs::Odometry &odom)
{
  POSITION.x = odom.pose.pose.position.x;
  POSITION.y = odom.pose.pose.position.y;
  POSITION.z = odom.pose.pose.position.z;

  float x = -odom.pose.pose.orientation.x;
  float y = odom.pose.pose.orientation.y;
  float z = -odom.pose.pose.orientation.z;
  float w = odom.pose.pose.orientation.w;

  ORIENTAION.pitch = -asin(2.0 * (x * z - w * y)) * 180.0 / PI;
  ORIENTAION.roll = atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z) * 180.0 / PI;
  ORIENTAION.yaw = atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z) * 180.0 / PI;
}

/* ROS Subcribers */
ros::Subscriber<sensor_msgs::Joy> joySub("joy", joyCallback);
ros::Subscriber<nav_msgs::Odometry> odomSub("camera/odom/sample", odomCallback); // TODO: Change to t265/odom/sample under deployment

/* Variable for the intervall time for walking case*/
Timer walkInterval;
uint8_t interval = 1; //1

/*  */
Timer XboxReadInterval;
uint8_t XboxInterval = 100;

/* Variable for storing loop time */
unsigned long loopTime;
unsigned long walkTime;

Timer idleTimer;
int idleTime = 10000;

/*------Variables for reading PID parameters from serial------*/
// Defining global variables for recieving data
//TODO Remove
float kp = 40.0f;  //STAND = 40.0f    moveToPoint = 
float ki = 0.01f; //STAND = 0.01f    moveToPoint =
float kd = 0.002f; //STAND = 0.002f   moveToPoint =
void changeConfigurations();

void setup()
{
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(joySub);
  nh.subscribe(odomSub);
  initializeButtons();
  initializeLights();
  initializeOdrives();
  initializeLegTracjetory();
  set_frequency(1.0f, autoParams);
  set_frequency(1.0f, manualParams);
  initializePIDs();
}

void loop()
{
  //loopTime = micros();
  nh.spinOnce();
  computePIDs();

  switch (currentState)
  {
  case S_IDLE:
    blinkLight(GREEN_LED);
    break;

  case S_STAND:
    blinkLight(ORANGE_LED);
    if (walkInterval.hasTimerExpired())
    {
      walkInterval.startTimer(interval);
      stand();
    }
    break;

  case S_CALIBRATE:
    calibrateOdriveMotors();
    calibrated = true;
    changeStateTo(S_IDLE);
    break;

  case S_READY:
    break;

  case S_PAUSE:
    blinkLight(RED_LED);
    break;

  case S_WALK:
    // walkTime = micros();
    if (walkInterval.hasTimerExpired())
    {
      walkInterval.startTimer(interval);
      locomotion(autoParams);

      /*--------------------------------------*/
      //Testing fuctions
      //layDown();
      //turnLeft();
      //turnRight();
    }
    // Serial.println(micros() - walkTime);
    break;

  case S_RUN:
    break;

  case S_JUMP:
  {
    jumpCommand();
  }

  break;

  case S_AUTONOMOUS:
    break;

  case S_MANUAL:
    if (XboxReadInterval.hasTimerExpired())
    {
      XboxReadInterval.startTimer(XboxInterval);
      readXboxButtons();
      mapXboxInputs();
    }

    if (walkInterval.hasTimerExpired())
    {
      walkInterval.startTimer(interval);
      locomotion(manualParams);
    }
    break;

  case S_BACKFLIP:
    break;

  case S_CONFIGURE:
    //readOdriveMotorCurrent();
    //Serial.println(motorcurrent[0][0]);
    break;

  case S_RESET:
    turnOffAllLights();
    checkForErrors();
    delay(200);
    resetMotorsErrors();
    delay(200);
    //writeConfig();;
    //setPreCalibrated(true);
    //rebootOdrives();
    //changeConfigurations();
    //saveConfigOdrives();
    //rebootOdrives();
    readConfig();
    changeStateTo(S_IDLE);
    break;

  case S_WARNING:
    blinkLight(ORANGE_LED);
    break;

  case S_ERROR:
    break;

  default:
    changeStateTo(S_IDLE);
    break;
  }
  readButtons();
}

void changeConfigurations()
{
  setLegMotorPID(kp, ki, kd);
  delay(500);
  setLegMotorTrapTraj(500,500,500);
  //newData = false;
  //}
}