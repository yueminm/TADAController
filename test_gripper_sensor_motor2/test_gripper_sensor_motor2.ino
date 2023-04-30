#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include <Servo.h>
Servo myservo;  // create servo object to control a servo

#include "DCMotor.h"
DCMotor motor1(33, 34, 31, 32,
              0.5, 0, 0,
              0.5, 1000, 80);
DCMotor motor2(36, 35, 30, 29,
              0.5, 0, 0,
              0.5, 1000, 80);

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
unsigned long startTime = 0;
uint16_t timingBudget = 15;


SFEVL53L1X distanceSensor;

// Modes
int executing = 0;
int finish = 0;
int calibrationPalm = 0;
int calibrationBall = 0;
int reset = 0;

// Initialization
// PD controller
const float Kp = 0.1;
const float Kd = 0.01;
float err_prev = 0.0;
float d_prev = 0;
float h_desired = 0;
int control_signal = 0;
int hInit = 0;
int distPalm = 0;
int distBall = 0;
int angleMax = 255;

// Helper functions
bool proportionalControl(double targetAng)
{
  double threshold = 0.1;
  double K_p = 10;
  double errorSignal = 0;
  int controlSignal = 0;  
  encData data = motor2.getPos();
  float theta = encToDeg(data.val);
  // Serial.print("enc ");
  // Serial.println(theta);
  Serial.print("target ");
  Serial.println(targetAng);

  errorSignal = theta-targetAng;

  bool reached = false;

  if (abs(errorSignal)<threshold)
  {
    controlSignal = 0;
    reached = true;
    // Serial.println("reached");
    // Serial.println(errorSignal);
    // Serial.println(theta);
    return(reached);
  }

  if (theta > angleMax)
  {
    controlSignal = 0;
  }

  else
  {
    controlSignal = (int)(K_p*-errorSignal);
    reached = false;
  }
  Serial.print("Control Signal \t");
  Serial.println(controlSignal);
  
  motor2.setSpeed(controlSignal);


  return(reached);
}

double getFingerDist(double ballHeight)
{
  double fingerDistInit = 250; //mm
  double fingerDistGrasp = 80;
  double ballHeightInit = 700;
  double ballHeightGrasp = 75; 
  double ballCaptureHeight = 60;
  double minFingerDist = 60;

  double m = (fingerDistGrasp-fingerDistInit) / (ballHeightGrasp-ballHeightInit);

  double c = fingerDistGrasp - m*ballHeightGrasp;
  if (ballHeight <= ballCaptureHeight)
  {
    return minFingerDist;
  }
  else
  {
    double fingerDist = m*ballHeight + c;
    return fingerDist;
  }
}

double getMotorAngle(double fingerDist)
{
  double motorAngle = -6.463e-05*pow(fingerDist,3) + 0.02792*pow(fingerDist,2) - 4.616*fingerDist+438.7;
  if (motorAngle < 0)
  {
    return 0;
  }
  if (motorAngle > 255)
  {
    return 255;
  }
  return motorAngle;
}



void setup(void)
{
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
  myservo.write(50);


  // Serial.begin(115200);
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
  // Serial.println("Before Motor Setup");
  executing = 0;
  finish = 0;

}

float encToDeg(int encVal) {
  return (float) (encVal) / (64.0 * 30.0) * 360.0;
}

void loop(void)
{
  encData data = motor2.getPos();
  float motorAngle = encToDeg(data.val);
  // Serial.print("enc ");
  // Serial.println(motorAngle);
  if (Serial.available() > 0){
    char input = Serial.read();
    if (input == 'o') 
    {
      myservo.write(50);
    }

    else if (input == 'p')
    {
      myservo.write(80);
    }

    else if (input == 'e')
    {
      calibrationBall = 0;
      executing = 1;
      reset = 0;
      motor2.resetEnc();
    }

    else if (input == 'q')
    {
      finish = 1;
      executing = 0;
    }

    else if (input == 'c')
    {
      calibrationPalm = 1;
    }

    else if (input == 'b')
    {
      calibrationPalm = 0;
      calibrationBall = 1;
    }

    else if (input == 'r')
    {
      executing = 0;
      finish = 0;
      reset = 1;
    }

  }

  if (finish == 1)
  {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  }

  if (calibrationPalm == 1)
  {
    Serial.println("Entered palm calibration");
    int sumDistance = 0;
    int numMeasurement = 10;
    for (int i = 0; i < numMeasurement; i++)
    {
      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      while (!distanceSensor.checkForDataReady())
      {
        // Serial.println("data not ready");        
        delay(10);
      }
      // Serial.println("after while loop");
      int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();
      sumDistance += distance;
    }
    distPalm = sumDistance / numMeasurement;
    Serial.print("Distance to palm:");
    Serial.print("\t");
    Serial.print(distPalm);
    calibrationPalm = 0;
  }

  if (calibrationBall == 1)
  {
    int sumDistance = 0;
    int numMeasurement = 10;
    for (int i = 0; i < numMeasurement; i++)
    {
      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      while (!distanceSensor.checkForDataReady())
      {
        delay(1);
      }
      int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();
      sumDistance += distance;
    }
    distBall = sumDistance / numMeasurement;
    Serial.print("Distance to ball:");
    Serial.print("\t");
    Serial.println(distBall);
    hInit = distPalm - distBall;
    calibrationBall = 0;
  }

  if (executing == 1)
    {
      myservo.write(50);
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
      Serial.print("\t");
      Serial.print(distance);
      Serial.print("\t");

      double ballHeight = distPalm - distance;

      if (ballHeight <= hInit*0.7)
      {
        // motor1.setSpeed(80);
        // motor2.setSpeed(160);
        // Serial.println("Motor Actuated");   
        // Convert ball height to target angle
        double targetFingerDist = getFingerDist(ballHeight);
        Serial.printf("target finger %f\n", targetFingerDist);
        double targetMotorAngle = getMotorAngle(targetFingerDist);
        proportionalControl(targetMotorAngle);    
        Serial.println("Motor Actuated");    
      }
  
      if (motorAngle >= angleMax)
      {
        motor2.setSpeed(0);
        Serial.println("Max Angle Reached");
        finish = 1;
      }

  if (finish == 1)
  {
    // proportionalControl(0);
  }

  if (reset == 1) {
    proportionalControl(0);
  }


      Serial.println();
  }
}
