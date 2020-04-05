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

ros::NodeHandle nh;

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

ros::Subscriber<sensor_msgs::Joy> joySub("joy", joyCallback);
ros::Subscriber<nav_msgs::Odometry> odomSub("camera/odom/sample", odomCallback); // TODO: Change to t265/odom/sample under deployment

/* Variable for the intervall time for walking case*/
Timer walkIntervall;
uint8_t intervall = 1; //1

/*  */
Timer XboxReadIntervall;
uint8_t XboxIntervall = 100;

/* Variable for storing loop time */
unsigned long loopTime;
unsigned long walkTime;

Timer idleTimer;
int idleTime = 10000;

/*------Variables for reading PID parameters from serial------*/
// Defining global variables for recieving data
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars]; // An array to store the received data
float kp = 25.0f;
float ki = 0.001f;
float kd = 0.0005f;
void readStringFromSerial();
void changeConfigurations();
String getValueFromSerial(String data, char separator, int index);

void setup()
{
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
  pinMode(13, OUTPUT);
}

void loop()
{
  //loopTime = micros();
  //readOdriveMotorPositions(hwSerials, odrives);
  //readOdriveMotorCurrent();
  nh.spinOnce();
  computePIDs();

  switch (currentState)
  {
  case S_IDLE:
    blinkLight(GREEN_LED);
    break;

  case S_STAND:
    blinkLight(ORANGE_LED);
    if (walkIntervall.hasTimerExpired())
    {
      walkIntervall.startTimer(intervall);
      stand();
    }

    break;

  case S_CALIBRATE:
    if (!calibrated)
    {
      calibrateOdriveMotors();
      calibrated = true;
    }
    changeStateTo(S_IDLE);
    break;

  case S_READY:
    break;

  case S_PAUSE:
    blinkLight(RED_LED);
    break;

  case S_WALK:
    // walkTime = micros();
    if (walkIntervall.hasTimerExpired())
    {
      walkIntervall.startTimer(intervall);
      locomotion(autoParams);

      /*--------------------------------------*/
      //Testing fuctions
      //layDown();
      //turnLeft();
      //turnRight();
    }
    //  Serial.print("Walk Time: ");
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
    if (XboxReadIntervall.hasTimerExpired())
    {
      XboxReadIntervall.startTimer(XboxIntervall);
      readXboxButtons();
      mapXboxInputs();
    }

    if (walkIntervall.hasTimerExpired())
    {
      walkIntervall.startTimer(intervall);
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
    //readConfig();
    //delay(500);
    //writeConfig();
    //setPreCalibrated(true);
    //saveConfigOdrives();
    //delay(500);
    //rebootOdrives();
    changeConfigurations();
    delay(200);
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

/**
  Reads a string from Serial Monitor.
*/
void readStringFromSerial()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while ((Serial.available() > 0) && (!newData))
  {
    rc = Serial.read();
    if (rc != endMarker)
    {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars)
      {
        ndx = numChars - 1;
      }
    }
    else
    {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
}
void changeConfigurations()
{
  if (newData)
  {
    kp = getValueFromSerial(receivedChars, ':', 0).toFloat();
    ki = getValueFromSerial(receivedChars, ':', 1).toFloat();
    kd = getValueFromSerial(receivedChars, ':', 2).toFloat();
    setLegMotorPID(kp, ki, kd);
    newData = false;
  }
}

String getValueFromSerial(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}