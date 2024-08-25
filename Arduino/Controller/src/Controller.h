#ifndef Controller_H
#define Controller_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <DueTimer.h>

// Encoders Pins
#define A_ENC1 22
#define B_ENC1 23

//Static Variables
#define pi 3.14159265
// Forward declaration of Encoder class
class Encoder;
class LPFilter;
// Dynamic Variables Decration
extern int lastEncoded_1;
extern int lastEncoded_2;
extern float pre_accel;
extern float speed;
extern int pre_v;
extern bool swing;

extern float K[4];
// objects
extern Encoder encoder1;
extern AccelStepper stepper;
extern LPFilter x_dot_f;
extern LPFilter theta_dot_f;
// Functions declaration
void readEncoder_J1();
void readEncoder_J2();
void stayHere();
void updatePos();
void stepperHandler();
void swingUp();
// ---------------- Encoder Class declaration
class Encoder {
private:
  float step;
  volatile float angle;
  float lastAngle;
  float velocity;
  long pre_time;
  bool changesFlage = false;
  // added
  const int windowSize = 3; // Window size (should be odd)
  float buffer[3];
  int bufferIndex = 0;
  float sum = 0;
  float smoothedValue[2] = {0, 0};
  long time[2] = {0, 0};

public:
  //construcor
  Encoder(int steps);
  //functions
  inline float getAngle() const;
  void setAngle(float ang);
  inline void move(bool dir);
  inline float getVelocity();
  inline float getAccel () const;
  inline bool isChanged();
};

inline float Encoder::getAngle() const{
  return angle;
  // return smoothedValue[1];
}
inline void Encoder::move(bool dir){
  (dir) ? angle+=step : angle-=step;
  changesFlage = true;

  long now = micros();
  sum -= buffer[bufferIndex];
  buffer[bufferIndex] = angle;
  sum += angle;
  bufferIndex = (bufferIndex + 1) % windowSize;
  smoothedValue[0] = smoothedValue[1]; 
  smoothedValue[1] = sum / windowSize;
  time[0] = time[1];
  time[1] = now;
} 
inline float Encoder::getVelocity(){
  if(!isChanged()) return 0;
  else{
    float w = (smoothedValue[1]-smoothedValue[0])*1e6/(time[1]-time[0]);
    return (w > 10 || w < -10) ? 0 : w;
  }
}
inline bool Encoder::isChanged(){
  if (!changesFlage) return false;
  else{
    changesFlage = false;
    return true;
  }
}
// ---------------- Encoder Class

// ---------------- LPFilter Class declaration
class LPFilter{
private:
  float rawData[2] = {0, 0};
  float lastFiltered = 0.0;
  float gain1 = 0.0;
  float gain2 = 0.0;
public:
  LPFilter(float gain);
  inline void filter(float data);
  inline float getFiltered();
};

inline void LPFilter::filter(float rawSignal){
  rawData[0] = rawData[1];
  rawData[1] = rawSignal;
  lastFiltered = gain1*lastFiltered + gain2*rawData[1];
}

inline float LPFilter::getFiltered(){
  return lastFiltered;
}
// ---------------- LPFilter Class 

#endif 