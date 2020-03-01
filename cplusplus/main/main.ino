/**
  The purpose of this project ...
  Libraries used:
  ArduinoOdrive - https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino
  ArduinoJSON - https://github.com/bblanchon/ArduinoJson
  -----------------------------------------------------------
  Code by: Magnus Kvendseth Øye, Vegard Solheim, Petter Drønnen
  Date: 10.02-2020
  Version: 0.4
  Website: https://github.com/magnusoy/Sparkie
*/

// Including libraries and headers
#include <ODriveArduino.h>
#include <ArduinoJson.h>
#include <SerialHandler.h>
#include <LegMovment.h>

//TODO make the dependency correct 
//#include "libraries/SerialHandler/SerialHandler.h"
//#include "libraries/LegMovment/LegMovment.h"

#include "Globals.h"
#include "Constants.h"
#include "OdriveParameters.h"
#include "IO.h"

SerialHandler serial(BAUDRATE, CAPACITY);
LegMovment legMovment;

/* Variable for storing time for leg tracjetory */
unsigned long n = 1;

void setup() {
  serial.initialize();
  initializeButtons();
  initializeLights();
  initializeOdrives();
  calibrateOdriveMotors(odrives);
}

void loop() {

  readOdriveMotorPositions(hwSerials, odrives);

  switch (currentState) {
    case S_IDLE:
      blinkLight(GREEN_LED);
      break;

    case S_CALIBRATE:

      break;

    case S_READY:

      break;

    case S_PAUSE:
      blinkLight(RED_LED);
      break;

    case S_WALK:
      for (int Odrive = 0; Odrive < 5; Odrive++) {
        double x = legMovment.stepX(n, LENGHT, FREQUENCY);
        double y = legMovment.stepY(n, AMPLITUDE, HEIGHT, FREQUENCY);
        for (int motor = 0; motor < 2; motor++) {
          double angle = legMovment.compute(x, y, motor);
          double motorCount = map(angle, -180, 180, -4096, 4096);
          setMotorPosition(Odrive, motor, motorCount);
        }
      }
      n += 1;
      break;

    case S_RUN:

      break;

    case S_JUMP:

      break;

    case S_AUTONOMOUS:

      break;

    case S_MANUAL:

      break;

    case S_BACKFLIP:

      break;

    case S_CONFIGURE:
      break;

    case S_RESET:
      turnOffAllLights();
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
}

/**
  Generate a JSON document and sends it
  over Serial.
*/
void sendJSONDocumentToSerial() {
  DynamicJsonDocument doc(220);
  //TODO make correct JSON
  doc["state"] = currentState;
  serial.write(doc);
}

/**
  Reads a JSON document from serial
  and decode it.
*/
void readJSONDocumentFromSerial() {
  JsonObject obj = serial.read();
  //TODO decode JSON
  //recCommand = obj["command"];
}
