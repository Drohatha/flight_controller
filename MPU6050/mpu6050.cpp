#include <wiringPiI2C.h>
#include <iostream>
#include <stdint.h>

#include "mpu6050.h"


// Math constants
#define gravity 9.81 // m/s^2 
#define pi 3.14

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
	first_iteration = true; 
	calculated_offset = false; 
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


	calculateOffset(); 
}

void MPU6050::readData(){
	// Acc data x axis
	acc_data_buffer[0] = wiringPiI2CReadReg8(fd_1, ACCEL_X_H);
	acc_data_buffer[1] = wiringPiI2CReadReg8(fd_1, ACCEL_X_L);
	acc_data_buffer[2] = wiringPiI2CReadReg8(fd_2, ACCEL_X_H);
	acc_data_buffer[3] = wiringPiI2CReadReg8(fd_2, ACCEL_X_L);
	//Acc data y axis
	acc_data_buffer[4] = wiringPiI2CReadReg8(fd_1, ACCEL_Y_H);
	acc_data_buffer[5] = wiringPiI2CReadReg8(fd_1, ACCEL_Y_L);
	acc_data_buffer[6] = wiringPiI2CReadReg8(fd_2, ACCEL_Y_H);
	acc_data_buffer[7] = wiringPiI2CReadReg8(fd_2, ACCEL_Y_L);
	//Acc data z axis
	acc_data_buffer[8] = wiringPiI2CReadReg8(fd_1, ACCEL_Z_H);
	acc_data_buffer[9] = wiringPiI2CReadReg8(fd_1, ACCEL_Z_L);
	acc_data_buffer[10] = wiringPiI2CReadReg8(fd_2, ACCEL_Z_H);
	acc_data_buffer[11] = wiringPiI2CReadReg8(fd_2, ACCEL_Z_L);
	//Gyro data x axis
	gyro_data_buffer[0] = wiringPiI2CReadReg8(fd_1,GYRO_X_H);
	gyro_data_buffer[1] = wiringPiI2CReadReg8(fd_1,GYRO_X_L);
	gyro_data_buffer[2] = wiringPiI2CReadReg8(fd_2,GYRO_X_H);
	gyro_data_buffer[3] = wiringPiI2CReadReg8(fd_2,GYRO_X_L);
	//Gyro data y axis
	gyro_data_buffer[4] = wiringPiI2CReadReg8(fd_1,GYRO_Y_H);
	gyro_data_buffer[5] = wiringPiI2CReadReg8(fd_1,GYRO_Y_L);
	gyro_data_buffer[6] = wiringPiI2CReadReg8(fd_2,GYRO_Y_H);
	gyro_data_buffer[7] = wiringPiI2CReadReg8(fd_2,GYRO_Y_L);
	//Gyro data z axis
	gyro_data_buffer[8] = wiringPiI2CReadReg8(fd_1,GYRO_Z_H);
	gyro_data_buffer[9] = wiringPiI2CReadReg8(fd_1,GYRO_Z_L);
	gyro_data_buffer[10] = wiringPiI2CReadReg8(fd_2,GYRO_Z_H);
	gyro_data_buffer[11] = wiringPiI2CReadReg8(fd_2,GYRO_Z_L);

	//Acc
	acc_merge_buffer[0] = acc_data_buffer[0] << 8| acc_data_buffer[1]; // X  from mpu 1
	acc_merge_buffer[1] = acc_data_buffer[2] << 8|  acc_data_buffer[3]; // X from mpu 2

	acc_merge_buffer[2] = acc_data_buffer[4] << 8| acc_data_buffer[5]; // Y from mpu 1
	acc_merge_buffer[3] = acc_data_buffer[6] << 8| acc_data_buffer[7]; // Y from mpu 2

	acc_merge_buffer[4] = acc_data_buffer[8] << 8| acc_data_buffer[9]; // Z from mpu 1
	acc_merge_buffer[5] = acc_data_buffer[10] << 8| acc_data_buffer[11]; // Z from mpu 2
	//Gyro
	gyro_merge_buffer[0] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // X from mpu 1
	gyro_merge_buffer[0] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // X from mpu 2

	gyro_merge_buffer[0] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // Y from mpu 1
	gyro_merge_buffer[0] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // Y from mpu 2

	gyro_merge_buffer[0] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // Z from mpu 1
	gyro_merge_buffer[0] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // Z from mpu 2


	//Convert to m/s^2
	//X axis 
	acc_raw[0] = acc_merge_buffer[0]*gravity/accScale;
	acc_raw[1] = acc_merge_buffer[1]*gravity/accScale;
	//Y axis MPU 1 & 2
	acc_raw[2] = acc_merge_buffer[2]*gravity/accScale;
	acc_raw[3] = acc_merge_buffer[3]*gravity/accScale;
	//Z axis MPU 1 & 2
	acc_raw[4] = acc_merge_buffer[4]*gravity/accScale;
	acc_raw[5] = acc_merge_buffer[5]*gravity/accScale;

	//Convert to degrees/s
	gyro_raw[0] = gyro_merge_buffer[0]*250/angularScale; 
	gyro_raw[1] = gyro_merge_buffer[1]*250/angularScale; 

	gyro_raw[2] = gyro_merge_buffer[2]*250/angularScale; 
	gyro_raw[3] = gyro_merge_buffer[3]*250/angularScale; 

	gyro_raw[4] = gyro_merge_buffer[4]*250/angularScale; 
	gyro_raw[5] = gyro_merge_buffer[5]*250/angularScale; 

	if(calculated_offset){
		for (int i = 0; i < 6; ++i){
			acc_raw[i] -= acc_offset[i];
			gyro_raw[i] -= gyro_offset[i];
		}


	}

	std::cout << "X from mpu 1 " << acc_raw[0] << "\t X from mpu 2 " << acc_raw[1] << std::endl; 
}

void MPU6050::calculateOffset(){
	int n = 1000; 
	for (int i = 0; i < n; ++i){
		readData(); 

		acc_offset[0] += acc_raw[0];
		acc_offset[1] += acc_raw[1];
		acc_offset[2] += acc_raw[2];
		acc_offset[3] += acc_raw[3];
		acc_offset[4] += acc_raw[4] - gravity;
		acc_offset[5] += acc_raw[5] - gravity;

		gyro_offset[0] += gyro_raw[0];
		gyro_offset[1] += gyro_raw[1];
		gyro_offset[2] += gyro_raw[2];
		gyro_offset[3] += gyro_raw[3];
		gyro_offset[4] += gyro_raw[4];
		gyro_offset[5] += gyro_raw[5];

	}
	for (int j = 0; j < 6; ++j){
		acc_offset[j] = acc_offset[j]/n;
		gyro_offset[j] = gyro_offset[j]/n; 
	}

	calculated_offset = true; 
}

void MPU6050::calculateEulerAngles(){
	/*
	float duration; 
	if(first_iteration){
		t_last = std::chrono::system_clock::now(); 
		first_iteration = false; 
	}
	//Transform the acc data to pitch and roll 
	float temp = sqrt(pow(accXYZ[1],2)+pow(accXYZ[2],2));
	roll_acc = 180*atan2(accXYZ[1],accXYZ[2])/pi; 
	pitch_acc = 180*atan2(-accXYZ[0],temp)/pi;


	t_now = std::chrono::system_clock::now(); 
	std::chrone::duration<double> elapsed_seconds = t_now-t_last; 
	duration = elapsed_seconds.count(); 
	t_last = t_now; 
*/


}




