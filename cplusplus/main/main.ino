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
//#include <SerialHandler.h>
//#include <LegMovment.h>

//TODO make the dependency correct
#include "src/libraries/SerialHandler/SerialHandler.h"
#include "src/libraries/LegMovement/LegMovement.h"

#include "Globals.h"
#include "Constants.h"
#include "OdriveParameters.h"
#include "IO.h"
SerialHandler serial(BAUDRATE, CAPACITY);
LegMovement legMovement;

/* Variable for storing time for leg tracjetory */
unsigned long n = 1;

void setup() {
  Serial.println("Setup started");
  serial.initialize();
  initializeButtons();
  initializeLights();
  initializeOdrives();
}

void loop() {

  //readOdriveMotorPositions(hwSerials, odrives);

  switch (currentState) {
    case S_IDLE:
      blinkLight(GREEN_LED);
      break;

    case S_CALIBRATE:
      calibrateOdriveMotors(odrives);
      changeStateTo(S_IDLE);
      break;

    case S_READY:

      break;

    case S_PAUSE:
      blinkLight(RED_LED);
      break;

    case S_WALK:
      for (int Odrive = 0; Odrive < 1; Odrive++) {
        double x = legMovement.stepX(n, LENGHT, FREQUENCY);
        double y = legMovement.stepY(n, AMPLITUDEOVER, AMPLITUDEUNDER, HEIGHT, FREQUENCY);
        //x = 0;
        //y = 200;
        for (int motor = 0; motor < 2; motor++) {
          double angle = legMovement.compute(x, y, motor);
          //Serial.println(angle);
          double motorCount = map(angle, -180, 180, -3000, 3000);
          setMotorPosition(Odrive, motor, motorCount);
        }
      }
      n += 5;
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
