/**
  The purpose of this project ...

  Libraries used:
  ArduinoOdrive - https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino
  ArduinoJSON - https://github.com/bblanchon/ArduinoJson
  -----------------------------------------------------------
  Code by: Magnus Kvendseth Øye, Vegard Solheim, Petter Drønnen
  Date: 06.01-2020
  Version: 0.1
  Website: https://github.com/magnusoy/Sparkie
*/

// Including libraries and headers
#include <ODriveArduino.h>
//#include <SerialHandler.h>
//#include <Timer.h>
//#include <ButtonTimer.h>
#include "Globals.h"
#include "Constants.h"
#include "OdriveParameters.h"


//SerialHandler serial(BAUDRATE, CAPACITY);

//HardwareSerial* hwSerials[4] ={&FRONT_LEFT, &FRONT_RIGHT, &BACK_LEFT, &BACK_RIGHT};
/*Odrive motordrivers, one Odrive per foot*/
ODriveArduino odriveFrontLeft(FRONT_LEFT);
ODriveArduino odriveFrontRight(FRONT_RIGHT);
ODriveArduino odriveBackLeft(BACK_LEFT);
ODriveArduino odriveBackRight(BACK_RIGHT);

ODriveArduino odrives[4] = {odriveFrontLeft, odriveFrontRight, odriveBackLeft, odriveBackRight};



void setup() {
  initializeIO();
  //serial.initialize();
}

void loop() {
  setOdrivesInControlMode(odrives);

  switch (currentState) {
    case S_IDLE:

      break;

    case S_CALIBRATE:

      break;

    case S_READY:

      break;

    case S_PAUSE:

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
