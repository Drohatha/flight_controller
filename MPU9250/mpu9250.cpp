#include <wiringPiI2C.h>
#include <iostream>
#include <stdint.h>
#include <unistd.h>

#include "mpu9250.h"


// Math constants
#define gravity 9.81 // m/s^2 
#define pi 3.14

#define powerConfig 107
#define powerCommand 0x00

#define accelConfig 0x1C
#define accelCommand 0x00 // +- 2g 
//#define accelCommand 0x08 // +- 4g
#define accScale 16384 //Number the MPU9250 yields when measured acc = 9.81

#define gyroConfig 0x1B
#define gyroCommand 0x00 // +- 250
//#define gyroCommand 0x08 // +- 500
#define angularScale 32768 // Number the MPU9250 yield when 250 deg/s is measured

#define bypassConfig 0x37
#define bypassCommand 0b00000010 // Enable bypass mode

#define magModeConfig 0x0A
#define magCommand 0b00010110 //Continuous measurement mode 2 and 16 bit measurement, poor documentation of what this actually is //0b00010110

// MPU9250 addresses
#define mpuAddress_1 0x68 
#define mpuAddress_2 0x69 

//Magnetometer address
#define magAddress 0x0C

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

//Magnetometer
#define MAG_X_L 0x03
#define MAG_X_H 0x04
#define MAG_Y_L 0x05
#define MAG_Y_H 0x06
#define MAG_Z_L 0x07
#define MAG_Z_H 0x08

#define MAG_STATUS 0x09



// Class constructor
MPU9250::MPU9250(){
	first_iteration = true; 
	calculated_offset = false; 
	init();
	std::cout << "Init complete" << std::endl; 
}

void MPU9250::init(){

	fd_1 = wiringPiI2CSetup(mpuAddress_1);
	fd_2 = wiringPiI2CSetup(mpuAddress_2);
	fd_3 = wiringPiI2CSetup(magAddress);
	//Enable first MPU9250
	wiringPiI2CWriteReg8(fd_1,powerConfig,powerCommand); // set power in
	wiringPiI2CWriteReg8(fd_1,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd_1,gyroConfig,gyroCommand); // set gyro scale
	wiringPiI2CWriteReg8(fd_1, bypassConfig,bypassCommand); // Set bypass mode
	//Enable second MPU9250
	wiringPiI2CWriteReg8(fd_2,powerConfig,powerCommand); // set power in
	wiringPiI2CWriteReg8(fd_2,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd_2, gyroConfig,gyroCommand); // set gyro scale
	
	wiringPiI2CWriteReg8(fd_2, bypassConfig,bypassCommand); // Set bypass mode
	wiringPiI2CWriteReg8(fd_3, magModeConfig,magCommand); 


	calculateOffset(); 
}

void MPU9250::readData(){

	//Vurder om man skal skrive det her til en for-løkke istedenfor

	// Acc data x axis

	for (int i = 0; i < 5; i++){
		acc_data_buffer[i] = wiringPiI2CReadReg8(fd_1, ACCEL_X_H + i); //Imu 1 Acc Write x_h, x_l, y_h, y_l, z_h,z_l into 0-5

		acc_data_buffer[i + 6] = wiringPiI2CReadReg8(fd_2, ACCEL_X_H + i): //Imu 2 Write x_h, x_l, y_h, y_l, z_h,z_l into 6-11
		
		gyro_data_buffer[i] = wiringPiI2CReadReg8(fd_1,GYRO_X_H + i); //Imu 1 gyro Write x_h, x_l, y_h, y_l, z_h,z_l into 0-5

		gyro_data_buffer[i + 6] = wiringPiI2CReadReg8(fd_2,GYRO_X_H + i); //Imu 2 gyro Write x_h, x_l, y_h, y_l, z_h,z_l into 6-11

	}

	for (int j = 0; j < 5 ; j++){
		mag_data_buffer[j] = wiringPiI2CReadReg8(fd_3,MAG_X_L + j) //Imu 1 mag Write x_l, x_h, y_l, y_h, z_l, z_h into 0-5
	}
	wiringPiI2CReadReg8(fd_3, MAG_STATUS);

	for (int m = 0; m < 5; m++){
		//Merge the lower and higher databits
		acc_merge_buffer[m] = acc_data_buffer[2*m] << 8| acc_data_buffer[2*m+1]; 
		gyro_merge_buffer[m] = gyro_data_buffer[2*m] << 8| gyro_data_buffer[2*m+1];
		acc_raw[m] = acc_merge_buffer[m]*gravity/accScale; 
		
		gyro_raw[m] = gyro_merge_buffer[m]*250/angularScale; // 250 deg/s is max measurement output
		mag_merge_buffer[m] = mag_data_buffer[m+1] << 8| mag_data_buffer[m]; // higher merged with lower
		mag_raw[m] = mag_merge_buffer[m]*0.15; 
	}

	std::cout << " Mag_x: " << mag_raw[0] << " Mag_y: " << mag_raw[1]
	<< " Mag_z " << mag_raw[2] << std::endl; 
	if(calculated_offset){
		for (int i = 0; i < 6; ++i){
			acc_raw[i] -= acc_offset[i];
			gyro_raw[i] -= gyro_offset[i];
		} 
	}
}

void MPU9250::calculateOffset(){
/*	int n = 1000; 
	for (int i = 0; i < n; ++i){

		readData(); 

		acc_offset[acc_x_mpu_1] += acc_raw[acc_x_mpu_1];
		acc_offset[acc_x_mpu_2] += acc_raw[acc_x_mpu_2];
		acc_offset[acc_y_mpu_1] += acc_raw[acc_y_mpu_1];
		acc_offset[acc_y_mpu_2] += acc_raw[acc_y_mpu_2];
		acc_offset[acc_z_mpu_1] += acc_raw[acc_z_mpu_1] - gravity;
		acc_offset[acc_z_mpu_2] += acc_raw[acc_z_mpu_2] - gravity;

		gyro_offset[gyro_x_mpu_1] += gyro_raw[gyro_x_mpu_1];
		gyro_offset[gyro_x_mpu_2] += gyro_raw[gyro_x_mpu_2];
		gyro_offset[gyro_y_mpu_1] += gyro_raw[gyro_y_mpu_1];
		gyro_offset[gyro_y_mpu_2] += gyro_raw[gyro_y_mpu_2];
		gyro_offset[gyro_z_mpu_1] += gyro_raw[gyro_z_mpu_1];
		gyro_offset[gyro_z_mpu_2] += gyro_raw[gyro_z_mpu_2];

	}
	for (int j = 0; j < 6; ++j){
		acc_offset[j] = acc_offset[j]/n;
		gyro_offset[j] = gyro_offset[j]/n; 
	}

	calculated_offset = true; */
}

*/
