#include "DCMotor.h"

long startTime;

typedef enum {
  FORWARD,
  BACKWARD,
  INPUTWAIT,
  PULL_I,
  PULL,
  STOP
} robotStates_e;

robotStates_e state = INPUTWAIT;

DCMotor motor(33, 34, 31, 32,
              0.5, 0, 0,
              0.5, 1000, 80);

void setup() {
  Serial.begin(38400);
  motor.setup();
  pinMode(14, INPUT);
}

void loop() {
  // int reading = analogRead(14);
  // Serial.println(reading);

  // motor.setSpeed(100);
  // delay(50);
  // motor.setSpeed(200);
  // delay(50);
  // motor.setSpeed(0);
  // delay(50);
  // motor.setSpeed(200);
  // delay(50);
  // motor.setSpeed(100);
  // delay(50);
  // encData enc = motor.getPos();
  // Serial.println(enc.val);
  // delay(50);
  robotStates_e nextState = state;
  Serial.println(state);
  switch(state) {
    case INPUTWAIT:
      motor.setSpeed(0);
      if (Serial.available()) {
        int a = 0;
        while (a != -1) {
          a = Serial.read();
        }
        nextState = PULL_I;
      }
      break;
    case PULL_I:
      motor.setSpeed(100);
      startTime = millis();
      nextState = PULL;
      break;
    case PULL:
      if (millis() - startTime > 2000) {
        Serial.print(startTime);
        Serial.print(" ");
        nextState = INPUTWAIT;
      }
      break;
    case STOP:
      motor.setSpeed(0);
      break;
  }
  state = nextState;
  delay(10);
}
