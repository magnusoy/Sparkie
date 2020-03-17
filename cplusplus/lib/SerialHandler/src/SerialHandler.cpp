#include "SerialHandler.h"
#include "ArduinoJson.h"

/**
 * SerialHandler is the constructor. 
 * Assigns the given parameters.
 * 
 * @param baudrate : 9600, 19200, 28800, 57600, 115200 ...
 * @param capacity : Size of the read JSON doc
 */
SerialHandler::SerialHandler(unsigned int baudrate, size_t capacity) {
  this->baudrate = baudrate;
  this->capacity = capacity;
}

/**
 * Initialize the Serial object.
 * 
 * @return true when its up,
 *         else false
 */ 
bool SerialHandler::initialize(void) {
  Serial.begin(this->baudrate);
  return Serial ? true : false;
}

/**
 * Reads and serialize the buffer to Json
 * document.
 * 
 * @return obj : document as JsonObject
 */ 
JsonObject SerialHandler::read(void) {
  JsonObject obj;
  if (Serial.available() > 0) {
    DynamicJsonDocument doc(this->capacity);
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) {
      deserializationError = true;
      return obj;
    }
    obj = doc.as<JsonObject>();
  }
  return obj;
}

/**
 * Clears the Serial buffer.
 */
void SerialHandler::flush(void) {
  if (Serial.available() > 0) {
    DynamicJsonDocument doc(this->capacity);
    DeserializationError error = deserializeJson(doc, Serial);
    if (error) {
      deserializationError = true;
      return;
    }
    //JsonObject obj = doc.as<JsonObject>();
  }
}

/**
 * Writes the given Json document to the Serial.
 * 
 * @param doc : DynamicJsonDocument
 */
void SerialHandler::write(DynamicJsonDocument doc) {
  serializeJson(doc, Serial);
  Serial.print("\n");
}
