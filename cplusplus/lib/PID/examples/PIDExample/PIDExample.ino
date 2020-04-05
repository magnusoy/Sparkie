#include <PID.h>

#define LL 0
#define LH 100
double target = 50;
double kp = 2.0;
double ki = 0.0;
double kd = 0.0;
PID pid(kp, ki, kd, REVERSE);

void setup() {
  Serial.begin(115200);
  pid.setUpdateTime(10);
  pid.setOutputLimits(LL, LH);
}

void loop() {
  double actual = random(0, 100);
  double output = pid.compute(actual, target);

  Serial.print(actual);
  Serial.print(",");
  Serial.print(target);
  Serial.print(",");
  Serial.println(output);
}
