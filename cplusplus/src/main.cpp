/**
  The purpose of this project ...
  
  Libraries used:
  ArduinoOdrive - https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino
  ROS - https://github.com/ros-drivers/rosserial
  -----------------------------------------------------------
  Code by: Magnus Kvendseth Øye, Vegard Solheim, Petter Drønnen
  Date: 08.04-2020
  Version: 0.9
  Website: https://github.com/magnusoy/Sparkie
*/

// Including libraries and headers
#include <Arduino.h>
#include "../lib/ODriveArduino/src/ODriveArduino.h"
#include "../lib/Timer/src/Timer.h"
#include "../lib/ros_lib/ros.h"
#include "../lib/ros_lib/nav_msgs/Odometry.h"
#include "../lib/ros_lib/sensor_msgs/Joy.h"
#include "../lib/ros_lib/geometry_msgs/Twist.h"
#include "../lib/ros_lib/std_msgs/UInt8.h"
#include "../lib/ros_lib/std_msgs/String.h"

#include "Globals.h"
#include "Constants.h"
#include "types.h"
#include "OdriveParameters.h"
#include "IO.h"
#include "XboxController.h"
#include "Locomotion.h"
#include "Navigation.h"

/* ROS Nodehandlers */
ros::NodeHandle nh;

void goalCallback(const std_msgs::UInt8 &goal)
{
  GOAL_REACHED = goal.data;
}

/**
 Navigation callback function for updating
 robot velocity inputs

 @param nav is the navigation msg
*/
void navCallback(const geometry_msgs::Twist &nav)
{
  NAVIGATION.VEL_LINEAR_X = nav.linear.x;
  NAVIGATION.VEL_LINEAR_Y = nav.linear.y;
  NAVIGATION.VEL_LINEAR_Z = nav.linear.z;

  NAVIGATION.VEL_ANGULAR_X = nav.angular.x;
  NAVIGATION.VEL_ANGULAR_Y = nav.angular.y;
  NAVIGATION.VEL_ANGULAR_Z = nav.angular.z;
}

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
  XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT = joy.axes[3];
  XBOX_CONTROLLER_INPUT.RJ_DOWN_UP = joy.axes[4];
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
ros::Subscriber<nav_msgs::Odometry> odomSub("t265/odom/sample", odomCallback);
ros::Subscriber<geometry_msgs::Twist> navSub("cmd_vel", navCallback);
ros::Subscriber<std_msgs::UInt8> goalSub("goal_reached", goalCallback);

std_msgs::String str_msg;

/* ROS Publisher */
ros::Publisher inPosition("in_position", &str_msg);
char take_img[2] = "1";
char dontTake_img[2] = "2";

/* Variable for the interval time for walking case*/
Timer moveTimer;
uint8_t moveInterval = 1; //1

/* Variable for storing loop time */
unsigned long loopTime;
unsigned long walkTime;

Timer transitionTimer;
int transitionTime = 5000;
Timer pictureTimer;
int pictureTime = 4000;

void setup()
{
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(joySub);
  nh.subscribe(odomSub);
  nh.subscribe(navSub);
  nh.subscribe(goalSub);
  nh.advertise(inPosition);
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

  switch (currentState)
  {

  case S_IDLE:
    blinkLight(GREEN_LED);
    break;

  case S_TRANSITION:
    blinkLight(RED_LED);
    if (moveTimer.hasTimerExpired())
    {
      moveTimer.startTimer(moveInterval);
      setIdlePosition();
    }
    if (!transition)
    {
      transitionTimer.startTimer(transitionTime);
      transition = true;
    }
    if (transitionTimer.hasTimerExpired())
    {
      changeStateTo(S_STAND);
    }
    break;

  case S_STAND:
    blinkLight(ORANGE_LED);
    computePIDs();
    if (moveTimer.hasTimerExpired())
    {
      moveTimer.startTimer(moveInterval);
      stand();
    }
    break;

  case S_LAYDOWN:
    blinkLight(BLUE_LED);
    layDown();
    break;

  case S_CALIBRATE:
    calibrateOdriveMotors();
    changeStateTo(S_IDLE);
    break;

  case S_TRANSITIONWALK:
    transitionToPoint(0, -180);
    if (!transition)
    {
      transitionTimer.startTimer(2000);
      transition = true;
    }
    if (transitionTimer.hasTimerExpired())
    {
      changeStateTo(nextState);
    }
    break;

  case S_WALK:
    // walkTime = micros();
    //computeHeight(autoParams);
    if (moveTimer.hasTimerExpired())
    {
      moveTimer.startTimer(moveInterval);
      locomotion(autoParams);
      /*--------------------------------------*/
      //Testing fuctions
      //layDown();
      //turnLeft();
      //turnRight();
    }
    // Serial.println(micros() - walkTime);
    break;

  case S_JUMP:
  {
    jumpCommand();
  }

  break;

  case S_AUTONOMOUS:
    //computeHeight(autoParams);
    mapNavigation();
    isGoalReached();
    if (moveTimer.hasTimerExpired())
    {
      moveTimer.startTimer(moveInterval);
      locomotion(autoParams);
    }
    break;

  case S_MANUAL:
    mapXboxInputs();
    //computePIDs();
    //computeHeight(manualParams);

    //float hei = Legs[0].getHeight();
    //char result[8];
    //dtostrf(val, 6, 2, result);
    //nh.loginfo(result);
    if (moveTimer.hasTimerExpired())
    {
      moveTimer.startTimer(moveInterval);
      locomotion(manualParams);
    }
    break;

  case S_RESET:
    //checkForErrors();
    //delay(10);
    //resetMotorsErrors();
    numberOfInspections = 1;
    changeStateTo(S_IDLE);
    break;

  case S_INSPECT:
    inspect();
    blinkLight(BLUE_LED);
    if (!transition)
    {
      transitionTimer.startTimer(5000);
      transition = true;
    }
    if (numberOfInspections == 2 || numberOfInspections == 4 || numberOfInspections == 5 || numberOfInspections == 6)
    {
      str_msg.data = dontTake_img;
      inPosition.publish(&str_msg);
      numberOfInspections++;
      changeStateTo(S_AUTONOMOUS);
    }
    if (transitionTimer.hasTimerExpired())
    {
      if (!pictureTaken)
      {
        str_msg.data = take_img;
        inPosition.publish(&str_msg);
        pictureTimer.startTimer(pictureTime);
        pictureTaken = true;
      }

      if (pictureTimer.hasTimerExpired())
      {
        numberOfInspections++;
        changeStateTo(S_TRANSITIONWALK);
        nextState = S_AUTONOMOUS;
      }
    }
    break;

  default:
    changeStateTo(S_IDLE);
    break;
  }
  readButtons();
  readXboxButtons();
}