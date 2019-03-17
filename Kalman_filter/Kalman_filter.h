#ifndef  _Kalman_filter_H_
#define  _Kalman_filter_H_
#include "Arduino.h"

class Kalman
{
  public:
  Kalman()
 {
  Q_angle=0.0001;
  Q_bias=0.003;
  R_measure=0.03;

  angle=0;
  bias=0;
  rate=0;

  P[0][0]=0;
  P[0][1]=0;
  P[1][0]=0;
  P[1][1]=0;
 }
  void setQangle(float Qangle); //covariance
  void setQbias(float Qbias); //covariance 
  void setRmeasure(float Rmeasure); //measurement noise
  float getAngle(float readAngle,float readRate, double dt);
  
 private:
 float Q_angle;
 float Q_bias;
 float R_measure;

 float angle;
 float bias;
 float rate;
 float P[2][2];
  
};

extern Kalman KalmanX;
extern Kalman KalmanY;

#endif