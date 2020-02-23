/**
  The purpose of this project ...
  Libraries used:
  ArduinoOdrive - https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino
  ArduinoJSON - https://github.com/bblanchon/ArduinoJson
  -----------------------------------------------------------
  Code by: Magnus Kvendseth Øye, Vegard Solheim, Petter Drønnen
  Date: 10.02-2020
  Version: 0.3
  Website: https://github.com/magnusoy/Sparkie
*/


// Including libraries and headers
#include <ODriveArduino.h>
#include<math.h>
#include <ArduinoJson.h>
//#include <SerialHandler.h>
//#include <Timer.h>
//#include <ButtonTimer.h>
#include "Globals.h"
#include "Constants.h"
#include "OdriveParameters.h"

/*  Variables used for blinking a led without delay*/
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 1000;

/* Variable for storing time for leg tracjetory */
unsigned long n = 1;


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
  initializeButtons();
  //initializeLights();
  //initializeIO();
  initializeOdrives();
  calibreateMotors();
  //setOdrivesInControlMode(odrives);
  Serial.println("Setup complete");

}

void loop() {

  //readOdriveMotorPositions(hwSerials, odrives);

  switch (currentState) {
    case S_IDLE:
      blinkLight(RUNNING_LED);
      Serial.println("State Idle");
      break;

    case S_CALIBRATE:

      break;

    case S_READY:

      break;

    case S_PAUSE:

      break;

    case S_WALK:
      //for (int j = 0; j < 5; j++) {
      //(time,amplitude,step lenght, frequency)
      double x = stepX(n, 20, 30, 2);
      double y = stepY(n, 20, 30, 2);
      Serial.print("X: ");
      Serial.println(x);
      Serial.print("Y: ");
      Serial.println(y);
      for (int i = 0; i < 2; i++) {
        double angle = inverseKinematicsLeg(x, y, i);
        Serial.print("angle: ");
        Serial.println(angle);
        double motorCount = map(angle, -180, 180, -4096, 4096);
        Serial.print("Motor count: ");
        Serial.println(motorCount);
        setMotorPosition(0, i, motorCount);
      }
      //}
      //delay(2000);
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

    case S_WARNING:
      blinkLight(WARNING_LED);
      break;

    case S_ERROR:
      blinkLight(ERROR_LED);
      break;

    default:
      changeStateTo(S_IDLE);
      break;

  }
  readButtons();
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
  for (int i = 0; i < 1; ++i) {
    hwSerials[i].begin(BAUDRATE);
  }
}

void initializeButtons() {
  pinMode(START_BTN, INPUT);
  pinMode(STOP_BTN, INPUT_PULLUP);
  pinMode(RESET_BTN, INPUT_PULLUP);
}

void initializeLights() {
  pinMode(RUNNING_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  pinMode(WARNING_LED, OUTPUT);
}

void blinkLight(int pin) {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(pin, ledState);
  }
}

void readButtons() {
  int btnState1 = digitalRead(START_BTN);
  //int btnState2 = digitalRead(STOP_BTN);
  //int btnState3 = digitalRead(RESET_BTN);
  //  if (btnState2) {
  //    changeStateTo(S_PAUSE);
  //  }
  //
  //  else if (btnState3) {
  //    changeStateTo(S_WARNING);
  //  }
  //  else if (btnState1) {
  //    changeStateTo(S_WALK);
  //  }

  if (btnState1) {
    changeStateTo(S_WALK);
  }
}

void setMotorPosition(const int odriveNumber, const int motorNumber, double pos) {
  odrives[odriveNumber].SetPosition(motorNumber, pos);
}


double inverseKinematicsLeg(double x, double y, int motor) {
  double alpha;
  double r = sqrt((x * x) + (y * y));
  double theta = atan(y / x);
  double gamma = acos((8100 + (r * r) - 25600) / (180 * r));
  double gammaInDegrees = gamma * 57.296;

  if (x < 0) {
    theta = theta - 3.14;
  }
  if (motor == INNER) {
    alpha = gamma - theta;
  } else if (motor == OUTER) {
    alpha = gamma + theta;
  }
  alpha = alpha * 57.296;
  return alpha;

}

double stepX(unsigned long t, int amp, int lenght, int f) {
  double x = lenght / 2 * sin(2 * 3.14 * f * t);
  return x;
}
double stepY(double t, int amp, int height, int f) {
  double y = -height + amp * cos(2 * 3.14 * f * t);
  return y;
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
  Calibreates motors.
  Be aware the motors will move during this process!
*/
void calibreateMotors() {
  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  odrives[0].run_state(INNER, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  odrives[0].run_state(INNER, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrives[0].run_state(INNER, requested_state, false); // don't wait
  delay(1000);
  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  odrives[0].run_state(OUTER, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  odrives[0].run_state(OUTER, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrives[0].run_state(OUTER, requested_state, false); // don't wait
  delay(1000);
}

/**
   Change the state of the statemachine to the new state
   given by the parameter newState
   @param newState The new state to set the statemachine to
*/
void changeStateTo(int newState) {
  currentState = newState;
}
