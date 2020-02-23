#include <InverseKinematics.h>

InverseKinematics IK_FrontLeftLeg;

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
  double x = IK_FrontLeftLeg.stepX(n, LENGHT, FREQUENCY);
  double y = IK_FrontLeftLeg.stepY(n, AMPLITUDE, HEIGHT, FREQUENCY);
  for (int m = 0; m < 2; ++m)
  {
    double angle = IK_FrontLeftLeg.compute(x, y, m);
    Serial.print("M: ");
    Serial.print(m);
    Serial.print(" | ");
    Serial.print(n);
    Serial.print(" | ");
    Serial.println(angle);
  }
  n += 1;
}