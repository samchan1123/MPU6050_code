/*
This is the cpp file
*/
#include<Wire.h>
#include"Arduino.h"
#include <MPU6050_HKIRS.h>

//Default Variable setting
uint8_t temp =MPU6050_TEMP_ON;
uint8_t clock=MPU6050_CLOCK_PLL_XGYRO;
uint8_t sleep=MPU6050_SLEEP_OFF;
uint8_t gyro_st=MPU6050_SELFTEST_OFF;
uint8_t acce_st=MPU6050_SELFTEST_OFF;
uint8_t gyro_sl=MPU6050_GYRO_FS_250;
uint8_t acce_sl=MPU6050_ACCEL_FS_2;
uint8_t hpf=MPU6050_HPF_0;
float acce_sl_factor=MPU6050_ACCEL_FA_2;
float gyro_sl_factor= MPU6050_GYRO_FA_250;

//axis
uint8_t x=120; //ASCII code
uint8_t y=121;
uint8_t z=122;

//Setting MPU6050 
void MPU6050_HKIRS::MPU6050_ADDRESS(uint8_t pin)
{
  if(pin==0)
  {
    address=LOW_ADDRESS;
  }
  if (pin==1)
  {
    address=HIGH_ADDRESS;
  }
}

void MPU6050_HKIRS::clockSource(uint8_t source)
{
  clock=source;
}

void MPU6050_HKIRS::temperature (uint8_t sensor)
{
  temp=sensor;
}

void MPU6050_HKIRS::sleepMode(uint8_t sleepmode)
{
  sleep=sleepmode;
}
void MPU6050_HKIRS::acce_hpf(uint8_t filter)
{
  hpf=filter;
}
void MPU6050_HKIRS::acce_scale(uint8_t scale,float factor)
{
  acce_sl=scale;
  acce_sl_factor=factor;
}
void MPU6050_HKIRS::gyro_scale(uint8_t scale, float factor)
{
  gyro_sl=scale;
  gyro_sl_factor=factor;
}
void MPU6050_HKIRS::acce_selfTest(uint8_t state)
{
  acce_st=state;
}
void MPU6050_HKIRS::gyro_selfTest(uint8_t state)
{
  gyro_st=state;
}


void MPU6050_HKIRS::initialization()
{
  //MPU6050
  uint8_t settingByte=0;
  settingByte|=clock;
  settingByte|=temp;
  settingByte|=sleep;
  Wire.beginTransmission(address);
  Wire.write(MPU6050_RA_PWR_MGMT_1);
  Wire.write(settingByte);
  Wire.endTransmission();
  
  //Acce
  settingByte=0;
  settingByte|=hpf;
  settingByte|=acce_sl;
  settingByte|=acce_st;
  Wire.beginTransmission(address);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(settingByte);
  Wire.endTransmission();
  
  //Gyro
  settingByte=0;
  settingByte|=gyro_sl;
  settingByte|=gyro_st;
  Wire.beginTransmission(address);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(settingByte);
  Wire.endTransmission();
  //timer=millis();
}

//Acceleration
float MPU6050_HKIRS::getAcce(uint8_t axis)
{
  float acceleration=0;
  //x-axis
  if (axis==x)
  {
  uint8_t xL,xH;
  Wire.beginTransmission(address);
  Wire.write(ACCE_X_L);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t) address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    xL=Wire.read();
  }
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(ACCE_X_H);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t) address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    xH=Wire.read();
  }
  Wire.endTransmission();
   acceleration= (xH<<8)|xL;
   acceleration=acceleration*acce_sl_factor;
   return (acceleration);
  } 
  //y-axis
  if (axis==y)
  {
  uint8_t yL,yH;
  Wire.beginTransmission(address);
  Wire.write(ACCE_Y_L);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    yL=Wire.read();
  }
  Wire.beginTransmission(address);
  Wire.write(ACCE_Y_H);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    yH=Wire.read();
  }
   acceleration= (yH<<8)|yL ;
   acceleration=acceleration*acce_sl_factor;
   return (acceleration); 
  }
  //y-axis
  if (axis==z)
  {
  uint8_t zL,zH;
  Wire.beginTransmission(address);
  Wire.write(ACCE_Z_L);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    zL=Wire.read();
  }
  Wire.beginTransmission(address);
  Wire.write(ACCE_Z_H);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    zH=Wire.read();
  }
   acceleration= (zH<<8)|zL;
   acceleration=acceleration*acce_sl_factor;
   return (acceleration);
  }
}
// Acc angle
float MPU6050_HKIRS:: getAccAngle(uint8_t axis)
{
  if(axis==x) // roll angle (angle rotate along x-axis) 
{ 
   return (atan2(MPU6050_HKIRS::getAcce(y),MPU6050_HKIRS::getAcce(z)))*180/PI;
}
  if(axis==y) // pitch angle (angle rotate along y-axis) 
{ 
   return (-atan2(MPU6050_HKIRS::getAcce(x),MPU6050_HKIRS::getAcce(z)))*180/PI;
}
 if(axis==z) // yaw angle (angle rotate along z-axis)  //not work
{ 
   return (atan2(MPU6050_HKIRS::getAcce(x),MPU6050_HKIRS::getAcce(y)))*180/PI;
}
}
//Acc angle with filter
float MPU6050_HKIRS::filterAccAngle(uint8_t axis,float LPF,float factor,float off_set)
{
  if(axis==x) // roll angle (angle rotate along x-axis) 
{ 
   acc_buffer[0]= MPU6050_HKIRS:: getAccAngle(x)*LPF+acc_buffer[1]*(1-LPF);
   acc_buffer[1]= acc_buffer[0]*factor;
   return acc_buffer[1]+off_set;
}
 if(axis==y) // pitch angle (angle rotate along y-axis) 
{ 
   acc_buffer[2]= MPU6050_HKIRS:: getAccAngle(y)*LPF+acc_buffer[2]*(1-LPF);
   acc_buffer[3]= acc_buffer[2]*factor;
   return acc_buffer[3]+off_set;
}
if(axis==z) // yaw angle (angle rotate along z-axis)  //not work
{ 
   acc_buffer[4]= MPU6050_HKIRS:: getAccAngle(z)*LPF+acc_buffer[4]*(1-LPF);
   acc_buffer[5]= acc_buffer[4]*factor;
   return acc_buffer[5]+off_set;
}
}
// angular velocity
float MPU6050_HKIRS::angular_v(uint8_t axis, float factor)
{
  float velocity=0;
 //roll (rotate along x)
  if(axis==x)
{
 uint8_t xL,xH=0;
  Wire.beginTransmission(address);
  Wire.write(GYRO_X_L);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t)address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    xL=Wire.read();
  }
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(GYRO_X_H);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t)address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    xH=Wire.read();
  }
  Wire.endTransmission();
   velocity= (xH<<8)|xL;
   velocity=velocity*gyro_sl_factor*factor;
   return (velocity);
}
//pitch (rotate along y)
 if(axis==y)
{
 uint8_t yL,yH=0;
  Wire.beginTransmission(address);
  Wire.write(GYRO_Y_L);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t)address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    yL=Wire.read();
  }
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(GYRO_Y_H);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t)address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    yH=Wire.read();
  }
  Wire.endTransmission();
   velocity= (yH<<8)|yL;
   velocity=velocity*gyro_sl_factor*factor;
   return (velocity);
}
 //yaw (rotate along z)
 if(axis==z)
{
 uint8_t zL,zH=0;
  Wire.beginTransmission(address);
  Wire.write(GYRO_Z_L);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t)address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    zL=Wire.read();
  }
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(GYRO_Z_H);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom((uint8_t)address,(uint8_t) 1);
  if(Wire.available()) 
  {  
    zH=Wire.read();
  }
  Wire.endTransmission();
   velocity= (zH<<8)|zL;
   velocity=velocity*gyro_sl_factor*factor;
   return (velocity);
}
}
//dt (measured time)
double MPU6050_HKIRS::dt()
{
  return(millis()-timer)*0.001; // converted to second
  
}
//reset timer after each measurement
void MPU6050_HKIRS::timer_reset()
{
  timer=millis();
}
float MPU6050_HKIRS::angular_mov(uint8_t axis, float factor)
{
 //roll (rotate along x)
  if(axis==x)
{
  gyro_buffer[0]=gyro_buffer[0]+(MPU6050_HKIRS::angular_v(x,factor)*MPU6050_HKIRS::dt());
  return gyro_buffer[0];
}
//pitch (rotate along y)
  if(axis==y)
{
  gyro_buffer[1]=gyro_buffer[1]+(MPU6050_HKIRS::angular_v(y,factor)*MPU6050_HKIRS::dt());
  return gyro_buffer[1];
}
//yaw (rotate along z)
  if(axis==z)
{
  gyro_buffer[2]=gyro_buffer[2]+(MPU6050_HKIRS::angular_v(z,factor)*MPU6050_HKIRS::dt());
  return gyro_buffer[2];
}
}
MPU6050_HKIRS IMU= MPU6050_HKIRS();
