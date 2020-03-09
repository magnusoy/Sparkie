#include <LegMovement.h>

LegMovement LM_LegMovement;

/* Parameters */
double AMPLITUDE = 20.0f;
double LENGHT = 30.0f;
double HEIGHT = 30.0f;
double FREQUENCY = 2.0f;

/* Stepnumber */
unsigned long n = 1;


void setup()
{
  Serial.begin(115200);
}

void loop()
{
  double x = LM_LegMovement.stepX(n, LENGHT, FREQUENCY);
  double y = LM_LegMovement.stepY(n, AMPLITUDE, HEIGHT, FREQUENCY);
  for (int m = 0; m < 2; ++m)
  {
    double angle = LM_LegMovement.compute(x, y, m);
    Serial.print("M: ");
    Serial.print(m);
    Serial.print(" | ");
    Serial.print(n);
    Serial.print(" | ");
    Serial.println(angle);
  }
  n += 1;
}
