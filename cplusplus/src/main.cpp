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
#include "../lib/ArduinoJson/src/ArduinoJson.h"
#include "../lib/ODriveArduino/src/ODriveArduino.h"
#include "../lib/SerialHandler/src/SerialHandler.h"
#include "../lib/Timer/src/Timer.h"

#include "Globals.h"
#include "Constants.h"
#include "types.h"
#include "OdriveParameters.h"
#include "IO.h"
#include "XboxController.h"
#include "Locomotion.h"

SerialHandler serial(BAUDRATE, CAPACITY);

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

void readXboxControllerInputs();

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
  serial.initialize();
  Serial.println("Setup started");
  initializeButtons();
  initializeLights();
  initializeOdrives();
  Serial.println("Setup finished");
  Serial.setTimeout(1);
  initializeLegTracjetory();
  set_frequency(1.0f, autoParams);
  set_frequency(1.0f, manualParams);
  initializePIDs();
}

void loop()
{
  //loopTime = micros();
  //readOdriveMotorPositions(hwSerials, odrives);
  //readOdriveMotorCurrent();
  readXboxControllerInputs();
  computePIDs();

  switch (currentState)
  {
  case S_IDLE:
    blinkLight(GREEN_LED);
    if (!idlePosition && calibrated)
    {
      setIdlePosition();
      idleTimer.startTimer(idleTime);
    }
    if (idleTimer.hasTimerExpired())
    {
      disarmMotors();
    }

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
      armMotors();
    }
    changeStateTo(S_STAND);
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
    //readXboxControllerInputs();
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
  serial.flush();
  //Serial.print("Loop Time: ");
  //Serial.println(micros() - loopTime);
}

/**
  Generate a JSON document and sends it
  over Serial.
*/
void sendJSONDocumentToSerial()
{
  DynamicJsonDocument doc(220);
  //TODO make correct JSON
  doc["state"] = currentState;
  serial.write(doc);
}

/**
  Reads a JSON document from serial
  and decode it.
*/
void readJSONDocumentFromSerial()
{
  //JsonObject obj = serial.read();
  //TODO decode JSON
  //recCommand = obj["command"];
}

/**
 * Maps the Serial input to corresponding variables
*/
void readXboxControllerInputs()
{
  if (Serial.available() > 0)
  {
    //const size_t capacity = JSON_ARRAY_SIZE(17) + JSON_ARRAY_SIZE(20);
    const size_t capacity = JSON_OBJECT_SIZE(32) + 512;
    DynamicJsonDocument doc(capacity);
    deserializeJson(doc, Serial);
    JsonObject obj = doc.as<JsonObject>();

    XBOX_CONTROLLER_INPUT.LJ_LEFT_RIGHT = obj["0"];
    XBOX_CONTROLLER_INPUT.LJ_DOWN_UP = obj["1"];
    XBOX_CONTROLLER_INPUT.LT = obj["2"];
    XBOX_CONTROLLER_INPUT.RJ_DOWN_UP = obj["3"];
    XBOX_CONTROLLER_INPUT.RJ_LEFT_RIGHT = obj["4"];
    XBOX_CONTROLLER_INPUT.RT = obj["5"];
    XBOX_CONTROLLER_INPUT.A = obj["6"];
    XBOX_CONTROLLER_INPUT.B = obj["7"];
    XBOX_CONTROLLER_INPUT.X = obj["8"];
    XBOX_CONTROLLER_INPUT.Y = obj["9"];
    XBOX_CONTROLLER_INPUT.LB = obj["10"];
    XBOX_CONTROLLER_INPUT.RB = obj["11"];
    XBOX_CONTROLLER_INPUT.MLB = obj["12"];
    XBOX_CONTROLLER_INPUT.MRB = obj["13"];
    XBOX_CONTROLLER_INPUT.MB = obj["14"];
    XBOX_CONTROLLER_INPUT.LJ = obj["15"];
    XBOX_CONTROLLER_INPUT.RJ = obj["16"];

    POSITION.x = obj["x"];
    POSITION.y = obj["y"];
    POSITION.z = obj["z"];

    ORIENTAION.pitch = obj["pitch"];
    ORIENTAION.roll = obj["roll"];
    ORIENTAION.yaw = obj["yaw"];
  }
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