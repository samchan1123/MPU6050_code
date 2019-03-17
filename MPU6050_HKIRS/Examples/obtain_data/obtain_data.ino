#include <MPU6050_HKIRS.h>
#include <Wire.h>

const float LPF=0.65;
const float x_factor=1.8;
const float y_factor=1.75;
const float x_offset=0;
const float y_offset=2;
void setup()
{
  Serial.begin(9600);
  
  IMU.MPU6050_ADDRESS(0); //define MPU6050-address
  IMU.gyro_scale(MPU6050_GYRO_FS_1000,MPU6050_GYRO_FA_1000); //edit setting of gyro
  IMU.initialization(); //Use default setting if no change 
}

void loop()
{
 Serial.print(IMU.angular_mov('y')); // obtain pitch angle by gyro
 IMU.timer_reset(); // needed to be reset after all measurement of gyro;
 Serial.print("   ");
 Serial.println(IMU.filterAccAngle('y',LPF,y_factor,y_offset)); //obtain pitch angle by accelerometer
 delay(5);// better no delay since integration of time is needed.
}

