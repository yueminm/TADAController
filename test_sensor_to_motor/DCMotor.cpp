#include "Arduino.h"
#include "DCMotor.h"

DCMotor::DCMotor(int pwm_, int dir_, int enc1_, int enc2_, 
                  double kp_, double ki_, double kd_, 
                  double diameter_, long ticksPerRotation_, long encThreshold_) {
  pwmPin = pwm_;
  dirPin = dir_;
  enc1 = enc1_;
  enc2 = enc2_;
  kp = kp_;
  ki = ki_;
  kd = kd_;
  diameter = diameter_;
  ticksPerRotation = ticksPerRotation_;
  encThreshold = encThreshold_;
  encP = new Encoder(enc1, enc2);
}

void DCMotor::setup() {
  pinMode(pwmPin, OUTPUT); // dc i1
  pinMode(dirPin, OUTPUT); // dc i2
  pinMode(enc1, INPUT_PULLUP); // dc i1
  pinMode(enc2, INPUT_PULLUP); // dc i2
}

void DCMotor::setSpeed(int pwmVal) {
  // find direction
  speedVal = pwmVal;
  if (pwmVal >= 0) { // forwards
    // limit motor output
    digitalWrite(dirPin, HIGH);
  } else { // backwards
    // limit motor output
    digitalWrite(dirPin, LOW);
  }

  pwmVal = abs(pwmVal);
  pwmVal = min(pwmVal, 255);
  analogWrite(pwmPin, pwmVal);
}

int DCMotor::getSpeed() {
  return speedVal;
}

encData DCMotor::getPos() {
  encData retval;
  long encVal = (*encP).read();
  retval.time = millis();
  retval.val = encVal;
  return retval;
}

void DCMotor::resetEnc() {
  encP->write(0);
}

// Return value: false when running, true when finished
bool DCMotor::doDriveDist() {
  bool retval = false;
  encData enc;
  long encPos;
  long dataTime;

  enc = getPos();
  encPos = enc.val;
  dataTime = enc.time;
  Serial.print("enc ");
  // Serial.println(target - encPos);
  Serial.print(target - encPos);
  Serial.print(" ");
  Serial.print(encPos);
  Serial.print(" ");
  Serial.print(prevPos);
  Serial.print(" ");
  Serial.println(encPos - prevPos);

  long dp = encPos - prevPos;
  long dt = dataTime - prevTime;
  if (dt == 0) dt = 1; // prevent divide by 0

  if (encPos == prevPos && abs(target - encPos) < encThreshold) { // to know when motor is done moving
    samePosCount++;
  } else {
    samePosCount = 0;
  }

  // if (abs(target - encPos) > encThreshold) {
  if (samePosCount < samePosThresh) {
    int pwmVal = (int) (kp * (target - encPos) - kd * ((float)dp/(float)dt));
    setSpeed(pwmVal);
  } else { // deadband for small error
    setSpeed(0);
    retval = true;
  }

  prevPos = encPos;
  prevTime = dataTime;
  return retval;
}

void DCMotor::setDriveDist(double dist) {
  long encoderTicks;
  long val = getPos().val;
  // Serial.print("start ");
  // Serial.println(val);

  // Direction determined by sign of dist
  encoderTicks = (long)(dist / (diameter * 3.14) * ticksPerRotation);
  target = val + encoderTicks;

  samePosCount = 0;
  prevPos = target;
  prevTime = 1;
}
