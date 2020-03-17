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
    t2.startTimer(1000);
    Serial.println("HIGH");
  } else if (t2.hasTimerExpired())
  {
    digitalWrite(LED, LOW);
    Serial.println("LOW");
    t1.startTimer(2000);
  }
  //Serial.println("Blinking without freezing loop");
}
