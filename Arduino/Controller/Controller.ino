#include "src/Controller.h"

Encoder encoder1(1440);
AccelStepper stepper(1, 25, 24);
LPFilter x_dot_f(0.9);
LPFilter theta_dot_f(0.85);

void stayHere();
void updatePos();
void stepperHandler();

void setup(){
  Serial.begin(250000);

  pinMode(A_ENC1, INPUT_PULLUP);
  pinMode(B_ENC1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A_ENC1), readEncoder_J1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_ENC1), readEncoder_J2, CHANGE);

  stepper.setMaxSpeed(25000); // Set maximum speed in steps per second
  stepper.setAcceleration(500000); // Set acceleration in steps per second per second
  stepper.setSpeed(0);
  encoder1.setAngle(-pi);
  swing = true;
  Timer5.attachInterrupt(stepperHandler).setFrequency(300);
  Timer4.attachInterrupt(updatePos).setFrequency(1000).start();
  Timer3.attachInterrupt(swingUp).setFrequency(300).start();
}

void loop(){ 
  if(swing) stepper.run();
  else stepper.runSpeed();
}