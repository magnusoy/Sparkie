#ifndef _SERIALHANDLER_
#define _SERIALHANDLER_
#include <ArduinoJson.h>

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class SerialHandler
{

public:
  SerialHandler(unsigned int baudrate, size_t capacity);
  bool initialize(void);
  void flush(void);
  JsonObject read(void);
  void write(DynamicJsonDocument doc);

private:
  size_t capacity;
  unsigned int baudrate;
  bool deserializationError;
};

#endif // _SERIALHANDLER_