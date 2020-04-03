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
}

void loop()
{
  //loopTime = micros();
  //readOdriveMotorPositions(hwSerials, odrives);
  //readOdriveMotorCurrent();
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
      //disarmMotors();
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
    if (walkIntervall.hasTimerExpired())
    {
      walkIntervall.startTimer(intervall);
      //locomotion(autoParams);

      /*--------------------------------------*/
      //Testing fuctions
      layDown();
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
      readXboxControllerInputs();
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
    //checkForErrors();
    //readConfig();
    //delay(500);
    //writeConfig();
    //setPreCalibrated(true);
    //saveConfigOdrives();
    //delay(500);
    //rebootOdrives();
    //delay(500);
    //readConfig();
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
  //serial.flush();
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
  }
}
