#include <wiringPiI2C.h>
#include <iostream>
#include <stdint.h>

#include "mpu6050.h"


// Math constants
#define gravity 9.81 // m/s^2 
#define PI 3.14

#define powerConfig 107
#define powerCommand 0x00

#define accelConfig 0x1C
#define accelCommand 0x00 // +- 2g 
//#define accelCommand 0x08 // +- 4g
#define accScale 16384 //Number the MPU6050 yields when measured acc = 9.81

#define gyroConfig 0x1B
#define gyroCommand 0x00 // +- 250
//#define gyroCommand 0x08 // +- 500
#define angularScale 32768 // Number the MPU6050 yield when 250 deg/s is measured



// MPU6050 addresses and commands
#define mpuAddress_1 0x68 
#define mpuAddress_2 0x69 


//Accelerometer 

#define ACCEL_X_H 59
#define ACCEL_X_L 60
#define ACCEL_Y_H 61
#define ACCEL_Y_L 62
#define ACCEL_Z_H 63
#define ACCEL_Z_L 64

// Gyro 

#define GYRO_X_H 67
#define GYRO_X_L 68
#define GYRO_Y_H 69
#define GYRO_Y_L 70 
#define GYRO_Z_H 71
#define GYRO_Z_L 72

// Class constructor
MPU6050::MPU6050(){

}

// Class init function? Merge with constructor? 
void MPU6050::init(){

	fd_1 = wiringPiI2CSetup(mpuAddress_1);
	fd_2 = wiringPiI2CSetup(mpuAddress_2); 
	//Enable first MPU6050
	wiringPiI2CWriteReg8(fd_1,powerConfig,powerCommand); // set power in
	wiringPiI2CWriteReg8(fd_1,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd_1,gyroConfig,gyroCommand); // set gyro scale
	//Enable second MPU6050
	wiringPiI2CWriteReg8(fd_2,powerConfig,powerCommand); // set power in
	wiringPiI2CWriteReg8(fd_2,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd_2, gyroConfig,gyroCommand); // set gyro scale

}

void MPU6050::read_data(){
	// Acc data x axis
	accDataBuffer[0] = wiringPiI2CReadReg8(fd_1, ACCEL_X_H);
	accDataBuffer[1] = wiringPiI2CReadReg8(fd_1, ACCEL_X_L);
	accDataBuffer[2] = wiringPiI2CReadReg8(fd_2, ACCEL_X_H);
	accDataBuffer[3] = wiringPiI2CReadReg8(fd_2, ACCEL_X_L);
	//Acc data y axis
	accDataBuffer[4] = wiringPiI2CReadReg8(fd_1, ACCEL_Y_H);
	accDataBuffer[5] = wiringPiI2CReadReg8(fd_1, ACCEL_Y_L);
	accDataBuffer[6] = wiringPiI2CReadReg8(fd_2, ACCEL_Y_H);
	accDataBuffer[7] = wiringPiI2CReadReg8(fd_2, ACCEL_Y_L);
	//Acc data z axis
	accDataBuffer[8] = wiringPiI2CReadReg8(fd_1, ACCEL_Z_H);
	accDataBuffer[9] = wiringPiI2CReadReg8(fd_1, ACCEL_Z_L);
	accDataBuffer[10] = wiringPiI2CReadReg8(fd_2, ACCEL_Z_H);
	accDataBuffer[11] = wiringPiI2CReadReg8(fd_2, ACCEL_Z_L);

	accMergeBuffer[0] = accDataBuffer[0] << 8 | accDataBuffer[1]; // X  from mpu 1
	accMergeBuffer[1] = accDataBuffer[2] << 8 | accDataBuffer[3]; // X from mpu 2

	accMergeBuffer[2] = accDataBuffer[4] << 8 | accDataBuffer[5]; // Y from mpu 1
	accMergeBuffer[3] = accDataBuffer[6] << 8 | accDataBuffer[7]; // Y from mpu 2

	accMergeBuffer[4] = accDataBuffer[8] << 8 | accDataBuffer[9]; // Z from mpu 1
	accMergeBuffer[5] = accDataBuffer[10] << 8 | accDataBuffer[11]; // Z from mpu 2

	std::cout << "X from mpu 1 " << accMergeBuffer[0] << "\t X from mpu 2" << accMergeBuffer[1] << std::endl; 

}

