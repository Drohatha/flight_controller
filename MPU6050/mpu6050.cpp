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
	//Gyro data x axis
	gyroDataBuffer[0] = wiringPiI2CReadReg8(fd_1,GYRO_X_H);
	gyroDataBuffer[1] = wiringPiI2CReadReg8(fd_1,GYRO_X_L);
	gyroDataBuffer[2] = wiringPiI2CReadReg8(fd_2,GYRO_X_H);
	gyroDataBuffer[3] = wiringPiI2CReadReg8(fd_2,GYRO_X_L);
	//Gyro data y axis
	gyroDataBuffer[4] = wiringPiI2CReadReg8(fd_1,GYRO_Y_H);
	gyroDataBuffer[5] = wiringPiI2CReadReg8(fd_1,GYRO_Y_L);
	gyroDataBuffer[6] = wiringPiI2CReadReg8(fd_2,GYRO_Y_H);
	gyroDataBuffer[7] = wiringPiI2CReadReg8(fd_2,GYRO_Y_L);
	//Gyro data z axis
	gyroDataBuffer[8] = wiringPiI2CReadReg8(fd_1,GYRO_Z_H);
	gyroDataBuffer[9] = wiringPiI2CReadReg8(fd_1,GYRO_Z_L);
	gyroDataBuffer[10] = wiringPiI2CReadReg8(fd_2,GYRO_Z_H);
	gyroDataBuffer[11] = wiringPiI2CReadReg8(fd_2,GYRO_Z_L);

	//Acc
	accMergeBuffer[0] = accDataBuffer[0] << 8| accDataBuffer[1]; // X  from mpu 1
	accMergeBuffer[1] = accDataBuffer[2] << 8|  accDataBuffer[3]; // X from mpu 2

	accMergeBuffer[2] = accDataBuffer[4] << 8| accDataBuffer[5]; // Y from mpu 1
	accMergeBuffer[3] = accDataBuffer[6] << 8| accDataBuffer[7]; // Y from mpu 2

	accMergeBuffer[4] = accDataBuffer[8] << 8| accDataBuffer[9]; // Z from mpu 1
	accMergeBuffer[5] = accDataBuffer[10] << 8| accDataBuffer[11]; // Z from mpu 2
	//Gyro
	gyroMergeBuffer[0] = gyroDataBuffer[0] << 8| gyroDataBuffer[1]; // X from mpu 1
	gyroMergeBuffer[0] = gyroDataBuffer[0] << 8| gyroDataBuffer[1]; // X from mpu 2

	gyroMergeBuffer[0] = gyroDataBuffer[0] << 8| gyroDataBuffer[1]; // Y from mpu 1
	gyroMergeBuffer[0] = gyroDataBuffer[0] << 8| gyroDataBuffer[1]; // Y from mpu 2

	gyroMergeBuffer[0] = gyroDataBuffer[0] << 8| gyroDataBuffer[1]; // Z from mpu 1
	gyroMergeBuffer[0] = gyroDataBuffer[0] << 8| gyroDataBuffer[1]; // Z from mpu 2


	//Convert to m/s^2
	accRaw[0] = accMergeBuffer[0]*gravity/accScale;
	accRaw[1] = accMergeBuffer[1]*gravity/accScale;
	//Y axis MPU 1 & 2
	accRaw[2] = accMergeBuffer[2]*gravity/accScale;
	accRaw[3] = accMergeBuffer[3]*gravity/accScale;
	//Z axis MPU 1 & 2
	accRaw[4] = accMergeBuffer[4]*gravity/accScale;
	accRaw[5] = accMergeBuffer[5]*gravity/accScale;

	//Convert to degrees/s
	gyroRaw[0] = gyroMergeBuffer[0]*250/angularScale; 
	gyroRaw[1] = gyroMergeBuffer[1]*250/angularScale; 

	gyroRaw[2] = gyroMergeBuffer[2]*250/angularScale; 
	gyroRaw[3] = gyroMergeBuffer[3]*250/angularScale; 

	gyroRaw[4] = gyroMergeBuffer[4]*250/angularScale; 
	gyroRaw[5] = gyroMergeBuffer[5]*250/angularScale; 

	

	std::cout << "X from mpu 1 " << accRaw[0] << "\t X from mpu 2 " << accRaw[1] << std::endl; 

}

