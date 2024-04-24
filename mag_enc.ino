#include <AS5145.h>
#include <PID_v1.h>

#define PIN_INPUT 9
#define PIN_OUTPUT 11
AS5145 myAS5145(9);
long value;
long avgval;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.25, Kd=0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
Serial.begin(9600);
 pinMode(9, INPUT);
Setpoint = 150;
 //turn the PID on
myPID.SetMode(1);
//myPID.SetSampleTime(500);
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  // myPID.SetOutputLimits(-125, 125);
}

void loop() {
// value= 0;
// for (int i=0; i<20; i++) {
//   value+= myAS5145.pwm_degrees();
//   delay(10);      // 50 readings per sec averaged down to 5 readings per sec
// }
// avgval= value/20;
//Serial.println(value);
value= myAS5145.pwm_degrees();

// Input = analogRead(PIN_INPUT);
Input= map(value, 0, 4095, 0, 255);
myPID.Compute();
analogWrite(PIN_OUTPUT, Output);
Serial.println(Input);

// value= myAS5145.pwm_degrees();
// Serial.println(value);

}


