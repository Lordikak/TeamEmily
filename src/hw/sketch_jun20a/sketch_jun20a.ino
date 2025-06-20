#include <VarSpeedServo.h>

VarSpeedServo myservo;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(7);
}

void loop() {
  // put your main code here, to run repeatedly:
  myservo.write(106+20, 30, true); // degrees, speed, true
  delay(1000);
  myservo.write(106, 30, true); // degrees, speed, true
  delay(1000);
}
