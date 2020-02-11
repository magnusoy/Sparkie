/**
  The purpose of this project ...

  Libraries used:
  ArduinoOdrive - https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino
  ArduinoJSON - https://github.com/bblanchon/ArduinoJson
  -----------------------------------------------------------
  Code by: Magnus Kvendseth Øye, Vegard Solheim, Petter Drønnen
  Date: 10.02-2020
  Version: 0.2
  Website: https://github.com/magnusoy/Sparkie
*/


// Including libraries and headers
#include <ODriveArduino.h>
#include <ArduinoJson.h>
//#include <SerialHandler.h>
//#include <Timer.h>
//#include <ButtonTimer.h>
#include "Globals.h"
#include "Constants.h"
#include "OdriveParameters.h"



/*Serial connection for each odrive*/
HardwareSerial hwSerials[4] = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

/*Odrive motordrivers, one Odrive per foot*/
ODriveArduino odriveFrontLeft(FRONT_LEFT);
ODriveArduino odriveFrontRight(FRONT_RIGHT);
ODriveArduino odriveBackLeft(BACK_LEFT);
ODriveArduino odriveBackRight(BACK_RIGHT);

ODriveArduino odrives[4] = {odriveFrontLeft, odriveFrontRight, odriveBackLeft, odriveBackRight};



void setup() {
  Serial.begin(BAUDRATE);

  initializeIO();
  initializeOdrives();
  //setOdrivesInControlMode(odrives);
}

void loop() {

  readOdriveMotorPositions(hwSerials, odrives);

  switch (currentState) {
    case S_IDLE:

      break;

    case S_CALIBRATE:

      break;

    case S_READY:

      break;

    case S_PAUSE:

      break;

    case S_WALK:

      break;
      transitionTo(S_RUN)

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

    case S_WARNING:

      break;

    case S_ERROR:

      break;

    default:

      break;
  }
}

/**
   Initialize buttons {2, 3, 4} as inputs.
   Initialize leds {9, 10, 11} as outputs.
*/
void initializeIO() {
  for (int i = 2; i < 5; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 9; i < 12; i++) {
    pinMode(i, OUTPUT);
  }
}

/**
  Initialize the four Odrives.
*/
void initializeOdrives() {
  for (int i = 0; i < 5; ++i) {
    hwSerials[i].begin(BAUDRATE);
  }
}

/**
  Generate a JSON document and sends it
  over Serial.
*/
void sendJSONDocumentToSerial() {
  DynamicJsonDocument doc(220);
  doc["state"] = currentState;
  serializeJson(doc, Serial);
  Serial.print("\n");
}


/**
  Reads content sent from the Teensy and
  flushes it, as it is for no use.
*/
void flushSerial() {
  if (Serial.available() > 0) {
    const size_t capacity = 15 * JSON_ARRAY_SIZE(2) + JSON_ARRAY_SIZE(10) + 11 * JSON_OBJECT_SIZE(3) + 520;
    DynamicJsonDocument doc(capacity);
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) {
      return;
    }
    JsonObject obj = doc.as<JsonObject>();
  }
}

/**
   Change the state of the statemachine to the new state
   given by the parameter newState
   @param newState The new state to set the statemachine to
*/
void changeStateTo(int newState) {
  currentState = newState;
}
