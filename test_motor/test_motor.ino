#include "DCMotor.h"

long startTime;
int turning = 0;
int controlSignal = 100;
int max_theta = 260;

typedef enum {
  FORWARD,
  BACKWARD,
  ANALOGIN,
  INPUTWAIT,
  PULL_I,
  PULL,
  STOP
} robotStates_e;

robotStates_e state = ANALOGIN;

const int CURR_PIN = 14;

DCMotor motor(33, 34, 31, 32,
              0.5, 0, 0,
              0.5, 1000, 80);
DCMotor motor2(36, 35, 30, 29,
              0.5, 0, 0,
              0.5, 1000, 80);

void setup() {
  Serial.begin(115200);
  motor.setup();
  motor2.setup();
  pinMode(14, INPUT);
  pinMode(CURR_PIN, INPUT);
}

float encToDeg(int encVal) {
  return (float) (encVal) / (64.0 * 30.0) * 360.0;
}

void loop() {
  // int reading = analogRead(14);
  // Serial.println(reading);

  // encData enc = motor.getPos();
  // Serial.println(enc.val);
  // delay(50);
  // robotStates_e nextState = state;
  // Serial.println(state);
  // switch(state) {
  //   case INPUTWAIT:
  //     motor.setSpeed(0);
  //     if (Serial.available()) {
  //       int a = 0;
  //       while (a != -1) {
  //         a = Serial.read();
  //       }
  //       nextState = PULL_I;
  //     }
  //     break;
  //   case PULL_I:
  //     motor.setSpeed(100);
  //     startTime = millis();
  //     nextState = PULL;
  //     break;
  //   case PULL:
  //     if (millis() - startTime > 2000) {
  //       Serial.print(startTime);
  //       Serial.print(" ");
  //       nextState = INPUTWAIT;
  //     }
  //     break;
  //   case STOP:
  //     motor.setSpeed(0);
  //     break;
  // }
  // state = nextState;
  // delay(10);
  encData data = motor2.getPos();
  // Serial.print("enc ");
  // Serial.println(encToDeg(data.val));
  int currSense = analogRead(CURR_PIN);
  // Serial.print("curr ");
  // Serial.println(currSense);
  float curr = ((float) currSense - 507) * 14.09;
  // Serial.println(curr);


  // Find when to start
  if (Serial.available() > 0){
    char input = Serial.read();
    if (input == 'w') 
    {
      turning = 1;
      motor2.resetEnc();
      // motor.setSpeed(100);
    }

    else if (input == 'p')
    {
      turning = 0;
      // motor.setSpeed(0);
    }
  }
  // Change motor speed
  if (turning == 1) 
    {
      // motor.setSpeed(controlSignal);
      motor2.setSpeed(controlSignal);
      // Serial.println(controlSignal);
      encData data = motor2.getPos();
      float theta = encToDeg(data.val);
      Serial.print("enc ");
      Serial.println(theta);
      if (theta >= max_theta)
      {
        Serial.println("max reached");
        Serial.println(theta);
        motor2.setSpeed(0);
        turning = 0;
      }

    }

  if (turning == 0)
  {
    motor.setSpeed(0);
    motor2.setSpeed(0);
  }
}
