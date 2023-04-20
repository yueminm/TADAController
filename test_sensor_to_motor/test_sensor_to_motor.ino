#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include "DCMotor.h"
DCMotor motor(33, 34, 31, 32,
              0.5, 0, 0,
              0.5, 1000, 80);

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
unsigned long startTime = 0;
uint16_t timingBudget = 15;

// PD controller
const float Kp = 0.1;
const float Kd = 0.05;
float err_prev = 0.0;
float d_prev = 0;
float h_desired = 1000.0;
int control_signal = 0;

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void setup(void)
{
  // Sensor
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

  distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(timingBudget);
  // distanceSensor.setDistanceModeLong();
  while (!Serial.available())
  {
    continue;
  }
  startTime = millis();


  // Motor
  motor.setup();
  pinMode(14, INPUT);  
}

void loop(void)
{
  if (millis() - startTime > 5000)
  {
    while (true){}
  }
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement

  // Sensor
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();

  distanceSensor.stopRanging();

  // Serial.print("Distance(mm): ");
  Serial.print(millis()-startTime);
  Serial.print('\t');
  Serial.print(distance);

  // Motor
  float h_curr = distance;
  if (h_curr >= h_desired)
  {
    while (true){}
  }
  else
  {
    float err_curr = h_curr - h_desired;
    float d_curr = err_curr - err_prev;
    control_signal = (int) Kp*err_curr + Kd*d_curr;
    if (control_signal < 0)
    {
      control_signal = 0;
    }
    if (control_signal > 255)
    {
      control_signal = 255;
    }
    err_prev = err_curr;
    d_prev = d_curr;
    motor.setSpeed(control_signal);
  }


  Serial.println();
}
