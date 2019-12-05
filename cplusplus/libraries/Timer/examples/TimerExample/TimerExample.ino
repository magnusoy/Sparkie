#include "Timer.h"

Timer t1;
Timer t2;

#define DURATION 1000

const int LED = 13;

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
}

void loop()
{
  if (t1.hasTimerExpired())
  {
    digitalWrite(LED, HIGH);
    t2.startTimer(DURATION);
  } else if (t2.hasTimerExpired())
  {
    digitalWrite(LED, LOW);
    t1.startTimer(DURATION);
  }
  Serial.println("Blinking without freezing loop");
}