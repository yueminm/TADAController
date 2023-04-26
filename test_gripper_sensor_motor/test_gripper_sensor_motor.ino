#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;

#include "DCMotor.h"
DCMotor motor1(33, 34, 31, 32,
               0.5, 0, 0,
               0.5, 1000, 80);
DCMotor motor2(36, 35, 29, 30,
               0.5, 0, 0,
               0.5, 1000, 80);

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
unsigned long startTime = 0;
uint16_t timingBudget = 15;

// PD controller
const float Kp = 0.1;
const float Kd = 0.01;
float err_prev = 0.0;
float d_prev = 0;
float h_desired = 0;
int control_signal = 0;

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
int sensing = 0;

void setup(void) {
  Wire.begin();
  Serial.begin(115200);
  // Motor
  motor1.setup();
  pinMode(14, INPUT);
  Serial.println("Motor Setup");

  // Sensor
  Wire.begin();

  // Gripper
  pinMode(12, OUTPUT);
  myservo.attach(12, 500, 2500);

  // Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(timingBudget);
  // distanceSensor.setDistanceModeLong();

  while (!Serial.available()) {
    continue;
  }
  startTime = millis();
  // Serial.println("Before Motor Setup");
}

void loop(void) {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'o') {
      myservo.write(50);
    }

    else if (input == 'p') {
      myservo.write(80);
    }

    else if (input == 'e') {
      sensing = 1;
    }
  }
  if (sensing == 1) {
    myservo.write(50);
    if (millis() - startTime > 30000) {
      motor1.setSpeed(0);
      while (true) {}
    }
    distanceSensor.startRanging();  //Write configuration bytes to initiate measurement

    // Sensor
    while (!distanceSensor.checkForDataReady()) {
      delay(1);
    }
    int distance = distanceSensor.getDistance();  //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();

    distanceSensor.stopRanging();

    // Serial.print("Distance(mm): ");
    Serial.print(millis() - startTime);
    Serial.print("\t");
    Serial.print(distance);
    Serial.print("\t");

    // Motor
    float h_curr = distance;
    float err_curr = h_curr - h_desired;
    float d_curr = err_curr - err_prev;
    control_signal = Kp * err_curr + Kd * d_curr;

    Serial.print("err_curr:  ");
    Serial.print(err_curr);
    Serial.print("\t");
    Serial.print("d_curr:  ");
    Serial.print(d_curr);
    Serial.print("\t");
    Serial.print("control_signal:  ");
    Serial.print(control_signal);
    Serial.print("\t");
    if (control_signal < 0) {
      control_signal = 0;
    }
    if (control_signal > 255) {
      control_signal = 255;
    }
    err_prev = err_curr;
    d_prev = d_curr;
    motor1.setSpeed(control_signal);
    Serial.println();
  }
}
