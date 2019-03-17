#include <Kalman_filter.h>
#include"Arduino.h"


void Kalman::setQangle(float Qangle)
{
  Q_angle=Qangle;
}

void Kalman::setQbias(float Qbias)
{
  Q_bias=Qbias;
}

void Kalman::setRmeasure (float Rmeasure)
{
  R_measure=Rmeasure;
}

float Kalman::getAngle(float readAngle,float readRate, double dt)
{
  // accurate velocity= angular_velocity-bias
  //equation angle= angle+ time*accurate velocity
  rate=readRate-bias;
  angle=angle+dt*rate;
  //prediction
  P[0][0]= P[0][0]+ dt*( dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1]=P[0][1]-dt*P[1][1];
  P[1][0]= P[1][0]-dt*P[1][1];
  P[1][1]= P[1][1]+Q_bias*dt;
  //estimate error
  float S= P[0][0]+ R_measure;
  //Kalman gain
  float K[2];
  K[0]= P[0][0]/S;
  K[1]= P[1][0]/S;
  //Calculate difference (measure & estimate)
  float Y= readAngle - angle;
  //Update output
  angle= angle+ Y*K[0];
  bias= bias+ Y*K[1];
  //Update covariance
  float P00= P[0][0]; //store estimate P[0][0]
  float P01= P[0][1];
  P[0][0]-= K[0]*P00;
  P[0][1]-= K[0]*P01;
  P[1][0]-= K[1]*P00;
  P[1][1]-= K[1]*P01;
  return angle;
}

Kalman KalmanX=Kalman();
Kalman KalmanY=Kalman();