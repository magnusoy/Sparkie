#include <SerialHandler.h>

#define BAUDRATE 9600
#define CAPACITY 255
Serialhandler serial(BAUDRATE, CAPACITY);

void setup() {
    serial.initialize();
}

void loop() {
    JsonObject obj = serial.read();
}