/*This is a .h file of MPU6050 (HKIRS version)*/
#ifndef _MPU6050_HKIRS_H_
#define _MPU6050_HKIRS_H_
#include"Arduino.h"
//Address
//MPU6050 Address
#define LOW_ADDRESS 0x68
#define HIGH_ADDRESS 0x69
//Gyro configuration
#define MPU6050_GYRO_CONFIG      0x1B
//Acce configuration
#define MPU6050_ACCEL_CONFIG     0x1C
//Setting address
#define MPU6050_RA_PWR_MGMT_1    0x6B
//Acce out address
#define ACCE_X_H 0x3B
#define ACCE_X_L 0x3C
#define ACCE_Y_H 0x3D
#define ACCE_Y_L 0x3E
#define ACCE_Z_H 0x3F
#define ACCE_Z_L 0x40
//Gyro out address
#define GYRO_X_H 0x43
#define GYRO_X_L 0x44
#define GYRO_Y_H 0x45
#define GYRO_Y_L 0x46
#define GYRO_Z_H 0x47
#define GYRO_Z_L 0x48
//Setting
//Clock source setting
#define MPU6050_CLOCK_INTERNAL          0b00000000  
#define MPU6050_CLOCK_PLL_XGYRO         0b00000001
#define MPU6050_CLOCK_PLL_YGYRO         0b00000010
#define MPU6050_CLOCK_PLL_ZGYRO         0b00000011
#define MPU6050_EXT_MHz                 0b00000100 //external 32.768kHz reference
#define MPU6050_EXT_kHz                 0b00000101 //external 19.2MHz reference
#define MPU6050_Reserved                0b00000110
#define MPU6050_STOPPED                 0b00000111
 /*
 range (for bit 0-2, start from bit 0 ):
0 Internal 8MHz oscillator
1 PLL with X axis gyroscope reference
2 PLL with Y axis gyroscope reference
3 PLL with Z axis gyroscope reference
4 PLL with external 32.768kHz reference
5 PLL with external 19.2MHz reference
6 Reserved
7 Stops the clock and keeps the timing generator in reset
 */
//Temperature
#define MPU6050_TEMP_ON             0b00000000
#define MPU6050_TEMP_OFF            0b00001000
//Sleep Mode
#define MPU6050_SLEEP_OFF           0b00000000 //default sleep on
#define MPU6050_SLEEP_ON            0b01000000
//Wake up

//Reset
#define MPU6050_RESET               0b10000000
//Self test (for both gyro an accele)
#define MPU6050_SELFTEST_OFF        0b00000000
#define MPU6050_SELFTEST_ON         0b11100000
//Sensitivity gyro range(degree/sec)
#define MPU6050_GYRO_FS_250         0b00000000
#define MPU6050_GYRO_FS_500         0b00001000
#define MPU6050_GYRO_FS_1000        0b00010000
#define MPU6050_GYRO_FS_2000        0b00011000
 /*
 range (for bit 3-4, start from bit 0 ):
 0 = +-250 degree, 131 LSB/ degree per s
 1 = +-500 degree, 65.5 LSB/ degree per s
 2 = +-1000 degree, 32.8 LSB/ degree per s
 3 = +-2000 degree, 16.4 LSB/ degree per s
 */
//gyro scale factor
#define MPU6050_GYRO_FA_250  0.0076335
#define MPU6050_GYRO_FA_500  0.015267
#define MPU6050_GYRO_FA_1000  0.030488
#define MPU6050_GYRO_FA_2000  0.060976

//Sensitivity accelerometer range(g)
#define MPU6050_ACCEL_FS_2          0b00000000
#define MPU6050_ACCEL_FS_4          0b00001000
#define MPU6050_ACCEL_FS_8          0b00010000
#define MPU6050_ACCEL_FS_16         0b00011000
/* 
 range (for bit 3-4, start from bit 0 ):
 0 = +-2g, 16384  LSB/mg 
 1 = +-4g, 8912 LSB/mg
 2 = +-8g, 4096 LSB/mg
 3 = +-16g, 2048 LSB/mg
 */
//Acce scale factor
#define MPU6050_ACCEL_FA_2         0.00024
#define MPU6050_ACCEL_FA_4         0.000897
#define MPU6050_ACCEL_FA_8         0.003906
#define MPU6050_ACCEL_FA_16         0.015625

//High pass filter
#define MPU6050_HPF_0               0b00000000
#define MPU6050_HPF_1               0b00000001
#define MPU6050_HPF_2               0b00000010
#define MPU6050_HPF_3               0b00000011
#define MPU6050_HPF_4               0b00000100
#define MPU6050_HPF_5               0b00000101
#define MPU6050_HPF_6               0b00000110
#define MPU6050_HPF_7               0b00000111
/*
 HPF**reset when used:
 0 = Reset (disable) 
 1 = On @ 5 Hz
 2 = On @ 2.5 Hz
 3 = On @ 1.25 Hz
 4 = On @ 0.63 Hz
 7 = Hold
*/


class MPU6050_HKIRS
{
 public:

 //setting functions
 void MPU6050_ADDRESS(uint8_t pin);
 void clockSource(uint8_t source);
 void sleepMode(uint8_t sleepmode);
 void temperature(uint8_t sensor);
 void acce_hpf(uint8_t filter);
 void acce_scale(uint8_t scale,float factor);
 void gyro_scale(uint8_t scale,float factor);
 void acce_selfTest(uint8_t state);
 void gyro_selfTest(uint8_t state);
 
 //initialise
 void initialization();
 
 //Data aquire
 float getAcce(uint8_t axis);
 float getAccAngle(uint8_t axis);
 float filterAccAngle(uint8_t axis,float LPF,float factor,float off_set);
 float angular_v(uint8_t axis,float factor);
 float angular_mov(uint8_t axis,float factor);
 double dt();
 void timer_reset();
 
 private:
 uint8_t address;
 float acc_buffer[6];
 float gyro_buffer[3];
 unsigned long timer; 
};

extern MPU6050_HKIRS IMU;

#endif // MPU6050_HKIRS