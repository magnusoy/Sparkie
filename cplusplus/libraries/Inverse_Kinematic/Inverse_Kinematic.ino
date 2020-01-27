#include<math.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  long x = 20;
  long y = 200;
  inverse_kinematics_right(x, y);
  delay(10000);
  //inverse_kinematics_left(x, y);

}

void inverse_kinematics_right(long x, long y) {
  double r = sqrt((x * x) + (y * y));
  double theta = atan(y / x);
  double gamma = acos(((90 * 90) + (r * r) - (160 * 160)) / (2 * 90 * r));
  double thetaInDegree = theta * 57296 / 1000;
  //gammaInDegree vi trenger:
  double gammaInDegree = gamma * 57296 / 1000;
  double alpha = theta + gamma;
  double x1 = 90 * cos(alpha);
  double y1 = 90 * sin(alpha);

  Serial.println(x1);
  Serial.println(y1);
}

void inverse_kinematics_left(long x, long y) {
  double r = sqrt((x * x) + (y * y));
  double theta = atan(y / x);
  double gamma = acos(((90 * 90) + (r * r) - (160 * 160)) / (2 * 90 * r));
  double thetaInDegree = theta * 57296 / 1000;
  //gammaInDegree vi trenger:
  double gammaInDegree = gamma * 57296 / 1000;
  double alpha = theta - gamma;
  double x1 = 90 * cos(alpha);
  double y1 = 90 * sin(alpha);

  Serial.println(x1);
  Serial.println(y1);
}
