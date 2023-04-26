#ifndef DCMotor_h
#define DCMotor_h

#include <Encoder.h>
#include <Servo.h>

typedef struct encoderReading {
  long val;
  long time;
} encData;

class DCMotor {
  public:
    DCMotor(int pwm, int dir, int enc1, int enc2, 
            double kp, double ki, double kd, 
            double diameter, long ticksPerRotation, long encThreshold);
    void setup();
    void setSpeed(int pwmVal);
    encData getPos();
    void setDriveDist(double distance);
    bool doDriveDist();
    int getSpeed();
    void resetEnc();
  private:
    int pwmPin, dirPin, enc1, enc2;
    long ticksPerRotation;
    float kp, ki, kd, diameter, Input, Output, Setpoint;
    Encoder *encP;
    int PIDCompute(int, int);
    long target;
    long encThreshold;
    int samePosCount;
    long samePosThresh = 10;
    long prevPos;
    long prevTime;
    int speedVal;
};

#endif // DCMotor_h