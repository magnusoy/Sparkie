#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

/** BUTTONS */
const uint8_t START_BTN = 2;
const uint8_t STOP_BTN = 3;
const uint8_t RESET_BTN = 4;

/** LED indicators */
const uint8_t RUNNING_LED = 9;
const uint8_t ERROR_LED = 10;
const uint8_t WARNING_LED = 11;

/** Hardware serials for Odrive */

#define FRONT_LEFT Serial1  // RX, TX (0, 1)
#define FRONT_RIGHT Serial2 // RX, TX (7, 8)
#define BACK_LEFT Serial3   // RX, TX (15, 14)
#define BACK_RIGHT Serial4  // RX, TX (21, 20)

/** JSON Serial */
#define BAUDRATE 115200 // Bits per second
#define CAPACITY 255 // Bits payload

#endif // _CONSTANTS_H_
