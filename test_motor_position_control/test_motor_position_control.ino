#include <Wire.h>

#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;

#include "DCMotor.h"
DCMotor motor1(33, 34, 31, 32,
              0.5, 0, 0,
              1, 64*30, 80);
DCMotor motor2(36, 35, 30, 29,
              0.5, 0, 0,
              1, 64*30, 80);

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

int sensing = 0;
int finish = 0;
int sweep = 0;

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

  // distanceSensor.setDistanceModeLong();
  
  while (!Serial.available())
  {
    continue;
  }
  startTime = millis();
  // Serial.println("Before Motor Setup");
  sensing = 0;
  finish = 0;

}

float encToDeg(int encVal) {
  return (float) (encVal) / (64.0 * 30.0) * 360.0;
}

void loop(void)
{
  encData data = motor2.getPos();
  float theta = encToDeg(data.val);
  Serial.print("enc ");
  Serial.println(theta);
  if (Serial.available() > 0){
    char input = Serial.read();
    if (input == 'o') 
    {
    }

    else if (input == 'p')
    {
    }

    else if (input == 'e')
    {
      sensing = 1;
      motor2.resetEnc();
    }

    else if (input == 's')
    {
      sweep = 1;
      motor2.resetEnc();
    }

    else if (input == 'q')
    {
      finish = 1;
      sensing = 0;
    }
  }

  if (finish == 1)
  {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  }
  if (sweep == 1)
  {
    for (int i=1;i<=27;i++)
    {
      double incAng = -10;
      double targAng = (double)(incAng*i);
      Serial.print("Target Angle\t");
      Serial.print(targAng);
      Serial.print("\t");
      int numreached = 0;
      while(numreached <= 10)
      {
        bool reached = proportionalControl(targAng);
        delayMicroseconds(90);
        if (reached==true)
        {
          numreached++;
        }
      }
      delay(2000);

      encData data = motor2.getPos();
      float theta = encToDeg(data.val);
      Serial.print("Angle: ");
      Serial.println(theta);

    }

    sweep = 0;

    while(true)
    {

    }    
  }


  if (sensing == 1)
    {
      // Serial.print("Distance(mm): ");
      Serial.print(millis()-startTime);
      Serial.print("\t");

      if (theta <= 240)
      {
        // motor1.setSpeed(80);
        //motor2.setSpeed(160);
        proportionalControl(-30);
        Serial.println("motor actuated");        
      }
  
      if (theta >= 240)
      {
        Serial.println("max reached");
        motor2.setSpeed(0);
        finish = 1;
      }

      // Motor
      // float h_curr = distance;
      // float err_curr = h_curr - h_desired;
      // float d_curr = err_curr - err_prev;
      // control_signal = Kp*err_curr + Kd*d_curr;
      
      // Serial.print("err_curr:  ");
      // Serial.print(err_curr);
      // Serial.print("\t");   
      // Serial.print("d_curr:  ");
      // Serial.print(d_curr);
      // Serial.print("\t");     
      // Serial.print("control_signal:  ");
      // Serial.print(control_signal);
      // Serial.print("\t");
      // if (control_signal < 0)
      // {
      //   control_signal = 0;
      // }
      // if (control_signal > 255)
      // {
      //   control_signal = 255;
      // }
      // err_prev = err_curr;
      // d_prev = d_curr;
      // motor1.setSpeed(control_signal);
      // motor2.setSpeed(control_signal);
      Serial.println();
    }
}

bool proportionalControl(double targetAng)
{
  double threshold = 0.1;
  double K_p = 10;
  double errorSignal = 0;
  int controlSignal = 0;  
  encData data = motor2.getPos();
  float theta = encToDeg(data.val);

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

  else
  {
    controlSignal = (int)(K_p*errorSignal);
    reached = false;
  }
  //Serial.print("Control Signal \t");
  //Serial.println(controlSignal);
  motor2.setSpeed(controlSignal);

  return(reached);


}


bool goToAngle(double targetAng)
{
  double encD = targetAng*3.14*30.0/360.0;
  Serial.print("Command \t");
  Serial.println(encD);
  motor2.setDriveDist(encD);
  bool move = motor2.doDriveDist();
  return(move);
}

