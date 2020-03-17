#ifndef ODriveErrors_h
#define ODriveErrors_h

#include "Arduino.h"

//variables to read errors from odrive
int axisError = 0;
int motorError = 0;
int controllerError = 0;
int encoderError = 0;

const char axisErrors[12][28] PROGMEM = {
    "INVALID_STATE",
    "DC_BUS_UNDER_VOLTAGE",
    "DC_BUS_OVER_VOLTAGE",
    "CURRENT_MEASUREMENT_TIMEOUT",
    "BRAKE_RESISTOR_DISARMED",
    "MOTOR_DISARMED",
    "MOTOR_FAILED",
    "SENSORLESS_ESTIMATOR_FAILED",
    "ENCODER_FAILED", // 8
    "CONTROLLER_FAILED",
    "POS_CTRL_DURING_SENSORLESS",
    "WATCHDOG_TIMER_EXPIRED"};
const char motorErrors[13][30] PROGMEM = {
    "PHASE_RESISTANCE_OUT_OF_RANGE",
    "PHASE_INDUCTANCE_OUT_OF_RANGE",
    "ADC_FAILED",
    "DRV_FAULT",
    "CONTROL_DEADLINE_MISSED",
    "NOT_IMPLEMENTED_MOTOR_TYPE",
    "BRAKE_CURRENT_OUT_OF_RANGE",
    "MODULATION_MAGNITUDE",
    "BRAKE_DEADTIME_VIOLATION",
    "UNEXPECTED_TIMER_CALLBACK",
    "CURRENT_SENSE_SATURATION",
    "INVERTER_OVER_TEMP",
    "CURRENT_UNSTABLE"};
const char controllerErrors[1][10] PROGMEM = {
    "OVERSPEED"};
const char encoderErrors[6][25] PROGMEM = {
    "UNSTABLE_GAIN",
    "CPR_OUT_OF_RANGE",
    "NO_RESPONSE", // 2
    "UNSUPPORTED_ENCODER_MODE",
    "ILLEGAL_HALL_STATE",
    "INDEX_NOT_FOUND_YET"};
//end error variables

#endif //ODriveErrors_h