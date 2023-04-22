#include <Servo.h>

Servo myservo;  // create servo object to control a servo

// 80: close
// 50: open

int pos = 0;
void setup() {
  pinMode(12, OUTPUT);
  myservo.attach(12, 500, 2500);
}

void loop() {
  // val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  // val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  // myservo.write(50);                  // sets the servo position according to the scaled value
  // delay(15);                           // waits for the servo to get there
  // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
  // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
  myservo.write(50);
}