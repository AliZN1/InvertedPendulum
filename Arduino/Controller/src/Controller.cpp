#include "Controller.h"

// Dynamic Variables Defenition
int lastEncoded_1;
int lastEncoded_2;
float pre_accel = 0;
float speed = 0;
int pre_v = 0;
bool swing = true;

// float K[] = {-7.162277660168376, -5.019869720674877, -16.553011730249953, -6.596947556022169};
// float K[] = {-1.1, -1.6, -4, -20};
float K[] = {-0.7, -1.4, -4, -20};

// Static Functions Defenition
void readEncoder_J1(){
  int encoded = (digitalRead(A_ENC1) << 1) | digitalRead(B_ENC1);
  int sum = (lastEncoded_1 << 2) | encoded;
  lastEncoded_1 = encoded;
  // Serial.println(sum, BIN);
  
  if (sum == 0b1001 || sum == 0b0110) encoder1.move(true);
  if (sum == 0b1100 || sum == 0b0011) encoder1.move(false);
}

void readEncoder_J2(){
  int encoded = (digitalRead(B_ENC1) << 1) | digitalRead(A_ENC1);
  int sum = (lastEncoded_2 << 2) | encoded;
  lastEncoded_2 = encoded;
  // Serial.println(sum, BIN);
  
  if (sum == 0b1001 || sum == 0b0110) encoder1.move(false);
  if (sum == 0b1100 || sum == 0b0011) encoder1.move(true);
}

void stepperHandler(){
  // float x =-1*desired_x -1*float(stepper.currentPosition())/3200*0.019*pi;
  float x =-1*float(stepper.currentPosition())/3200*0.019*pi; // Convert steppers to meter
  float x_dot = x_dot_f.getFiltered(); // Use the filtered linear velocity of the cart
  float theta = encoder1.getAngle(); // Get the current angular position
  float theta_dot = theta_dot_f.getFiltered(); // Use the filtered angular velocity of the pendulum

  float signal = x*K[0] + x_dot*K[1] + theta*K[2] + theta_dot*K[3]; // Sum the weighted states to calculate the total required force for controlling the next state
  float accel = signal*10.2382*10000; // Convert force to acceleration 

  speed += (pre_accel+accel)*0.5*0.01; // Integrate the acceleration to obtain velocity
  pre_accel = accel;

  if(speed < -22000) speed = -22000; // Check the upper and lower limits for motor speed
  else if(speed > 22000) speed = 22000;

  // Serial.print(x, 4);
  // Serial.print(", ");
  // Serial.print(x_dot, 4);
  // Serial.print(", ");
  // Serial.print(theta, 6);
  // Serial.print(", ");
  // Serial.print(theta_dot, 4);
  // Serial.print(", ");
  // Serial.println(signal, 4);
  // Serial.println(speed);

  stepper.setSpeed(speed); // Set the new motor speed
  if (x < -0.17 || x > 0.17){ // Ensure the cart does not exceed the limits
    stayHere();
  } 
}

void updatePos(){ // This function updates the filtered signals
  x_dot_f.filter(-1*stepper.speed()/3200*0.019);
  theta_dot_f.filter(encoder1.getVelocity());
}

void swingUp(){ // This function runs at the beginning of the code to swing up the pendulum
  float theta = encoder1.getAngle();
  if(theta > -2*pi-0.05 && theta < -2*pi+0.05){ // Switch to the LQR controller if the angle reaches the limit, swung up from right side
    swing = false;
    encoder1.setAngle(theta+2*pi+0.005);
    Timer3.stop();
    Timer5.start();
  }
  else if(theta > -0.05 && theta < 0.05){ // Switch to the LQR controller if the angle reaches the limit, swung up from left side
    swing = false;
    encoder1.setAngle(theta-0.005);
    Timer3.stop();
    Timer5.start();
  }
  else{ // Keep swinging by moving the cart a specific distance when the pendulum is swinging back, but before it passes the lowest balance point
    int v = int(-1*theta_dot_f.getFiltered()*3200/pi);
    int pos = 2022;
    if (pre_v < -1 && v > 1) stepper.moveTo(-pos);
    else if(pre_v > 1 && v < -1) stepper.moveTo(pos);
    pre_v = v;
  }
}

void stayHere(){ // Used to lock the code
  // stepper.stop();
  delay(10);
  // Timer5.stop();
  while(true){}
}

// Classes Function Defenition 
// --------------------------------- Encoder class defenition
Encoder::Encoder(int steps) : step(2*pi/steps){}
void Encoder::setAngle(float ang){
  angle = ang;
  lastAngle = ang;
  smoothedValue[0] = ang;
  smoothedValue[1] = ang;
}
// --------------------------------- Encoder class

// --------------------------------- PLFilter class defenition
LPFilter::LPFilter(float gain){
  if (gain > 0 && gain < 1){
    gain1 = gain;
    gain2 = (1 - gain);
  }
  else{
    gain1 = 1;
    gain2 = 0;
  }
}
// --------------------------------- PLFilter class 

