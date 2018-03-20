#include <wiringPiI2C.h>
#include <iostream>
#include <stdint.h>
#include <unistd.h>

//#include <chrono>
//#include <ctime>

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



enum axis{
	x=0,
	y,
	z
};
enum imu{
	imu_1=0,
	imu_2
};


enum mag{
	mag_x_mpu_1=0,
	mag_x_mpu_2,
	mag_y_mpu_1,
	mag_y_mpu_2,
	mag_z_mpu_1,
	mag_z_mpu_2
};

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
	wiringPiI2CWriteReg8(fd_3, magModeConfig,magCommand); //Start mag and continous measurement mode 


	calculateOffset(); 
}

void MPU9250::readData(){


	for (int i = 0; i < 6; i++){
		acc_data_buffer[imu_1][i] = wiringPiI2CReadReg8(fd_1, ACCEL_X_H + i); //Imu 1 Acc Write x_h, x_l, y_h, y_l, z_h,z_l into 0-5
		acc_data_buffer[imu_2][i] = wiringPiI2CReadReg8(fd_2, ACCEL_X_H + i); //Imu 2 Write x_h, x_l, y_h, y_l, z_h,z_l into 6-11
		
		gyro_data_buffer[imu_1][i] = wiringPiI2CReadReg8(fd_1,GYRO_X_H + i); //Imu 1 gyro Write x_h, x_l, y_h, y_l, z_h,z_l into 0-5
		gyro_data_buffer[imu_2][i] = wiringPiI2CReadReg8(fd_2,GYRO_X_H + i); //Imu 2 gyro Write x_h, x_l, y_h, y_l, z_h,z_l into 6-11
	}

	for (int i = 0; i < 3; i++){ // 3 is num of axis

		//Could have, for each imu = 0
		acc_merge_buffer[imu_1][i] = acc_data_buffer[imu_1][2*i] << 8| acc_data_buffer[imu_1][2*i+1];
		acc_merge_buffer[imu_2][i] = acc_data_buffer[imu_2][2*i] << 8| acc_data_buffer[imu_2][2*i+1];

		gyro_merge_buffer[imu_1][i] = gyro_data_buffer[imu_1][2*i] << 8| gyro_data_buffer[imu_1][2*i+1];
		gyro_merge_buffer[imu_2][i] = gyro_data_buffer[imu_2][2*i] << 8| gyro_data_buffer[imu_2][2*i+1];

		acc_raw[imu_1][i] = acc_merge_buffer[imu_1][i]*gravity/accScale; 
		acc_raw[imu_2][i] = acc_merge_buffer[imu_2][i]*gravity/accScale;

		gyro_raw[imu_1][i] = gyro_merge_buffer[imu_1][i]*250.0/angularScale; // 250 degreee/s / LSB 
		gyro_raw[imu_2][i] = gyro_merge_buffer[imu_2][i]*250.0/angularScale; 
	}
	
	std::cout << " Acc x " << gyro_raw[imu_2][x] << " Acc y " << gyro_raw[imu_2][y] << " Acc z: " << gyro_raw[imu_2][z] << std::endl; 

	
	/* Dont read mag data now! Check if the other data make sence! 
	mag_data_buffer[imu_1][0] = wiringPiI2CReadReg8(fd_3,MAG_X_H);
    mag_data_buffer[imu_1][1] = wiringPiI2CReadReg8(fd_3,MAG_X_L);

    mag_data_buffer[imu_1][2] = wiringPiI2CReadReg8(fd_3,MAG_Y_H);
    mag_data_buffer[imu_1][3] = wiringPiI2CReadReg8(fd_3,MAG_Y_L);

    mag_data_buffer[imu_1][4] = wiringPiI2CReadReg8(fd_3,MAG_Z_H);
    mag_data_buffer[imu_1][5] = wiringPiI2CReadReg8(fd_3,MAG_Z_L);
    wiringPiI2CReadReg8(fd_3, MAG_STATUS);

    mag_merge_buffer[imu_1][x] = mag_data_buffer[0] << 8| mag_data_buffer[1];
	mag_merge_buffer[imu_1][y] = mag_data_buffer[2] << 8| mag_data_buffer[3];
	mag_merge_buffer[imu_1][z] = mag_data_buffer[4] << 8| mag_data_buffer[5];

	std::cout << "Mag x dir: " << mag_merge_buffer[mag_x_mpu_1] << std::endl;

	mag_raw[imu_1][x] = mag_merge_buffer[imu_1][x]*0.15; //0.15 micro Tesla pr LSB 
    mag_raw[imu_1][y] = mag_merge_buffer[imu_1][y]*0.15;
    mag_raw[imu_1][z] = mag_merge_buffer[imu_1][z]*0.15; 

    std::cout << " Mag_x: " << mag_raw[mag_x_mpu_1] << " Mag_y: " << mag_raw[mag_y_mpu_1]
    << " Mag_z " << mag_raw[mag_z_mpu_1] << std::endl;
    */
	if(calculated_offset){
		for (int i = 0; i < 3; ++i){
			//acc_raw[imu_1][i] -= acc_offset[imu_1][i];
			//gyro_raw[imu_1][i] -= gyro_offset[imu_1][i];
			acc_raw[imu_2][i] -= acc_offset[imu_2][i];
			gyro_raw[imu_2][i] -= gyro_offset[imu_2][i];

		} 
		
	}
}
	
void MPU9250::calculateOffset(){


	int n = 1000; 
	for (int i = 0; i < n; ++i){
		//Read 1000 messages! 
		readData(); 

		acc_offset[imu_1][x] += acc_raw[imu_1][x];
		acc_offset[imu_2][x] += acc_raw[imu_2][x];
		acc_offset[imu_1][y] += acc_raw[imu_1][y];
		acc_offset[imu_2][y] += acc_raw[imu_2][y];
		acc_offset[imu_1][z] += acc_raw[imu_1][z] - gravity;
		acc_offset[imu_2][z] += acc_raw[imu_2][z] - gravity;


		gyro_offset[imu_1][x] += gyro_raw[imu_1][x];
		gyro_offset[imu_2][x] += gyro_raw[imu_2][x];
		gyro_offset[imu_1][y] += gyro_raw[imu_1][y];
		gyro_offset[imu_2][y] += gyro_raw[imu_2][y];
		gyro_offset[imu_1][z] += gyro_raw[imu_1][z];
		gyro_offset[imu_2][z] += gyro_raw[imu_2][z];

	}
	for (int j = 0; j < 3; j++){

		acc_offset[imu_1][j] = acc_offset[imu_1][j]/n; 
		acc_offset[imu_2][j] = acc_offset[imu_2][j]/n;

		gyro_offset[imu_1][j] = gyro_offset[imu_1][j]/n; 
		gyro_offset[imu_2][j] = gyro_offset[imu_2][j]/n;
	}
	calculated_offset = true;
}
/*
void MPU9250::calibrateMagnetometer(){
	auto start_time = std::chrono::system_clock::now(); 
	auto now = std::chrono::system_clock::now(); 
	std::chrono::duration<double> elapsed_time = now - start_time; 
	
	enum imu{
		imu_1,
		imu_2
	}; 
	enum max_readings{
		x_1,
		x_2,
		y_1,
		y_2,
		z_1,
		z_2
	};
	
	double max[2][6]; // Two imu's and 6 values we want to find 
	double min[2][6]; // Two imu's and 6 values we want to find
	//Rotate around x axis
	
	readData(); 
		
	max[imu_1][y_1] = mag_raw[1]; // Improve code quality here
	max[imu_1][z_1] = mag_raw[2]; 


	while(elapsed_time < 10.0){
		readData();
		//Find y max and y min
		if(mag_raw[1] > max[imu_1][y_1]){
			max[imu_1][y_1] = mag_raw[1];
		}else if(mag_raw[1] < min[imu_1][y_1]){
			min[imu_1][y_1] = mag_raw[1];
		}
		//Find z max and z min
		if(mag_raw[2] > max[imu_1][z_1]){
			max[imu_1][z_1] = mag_raw[2]; 
		}else if(mag_raw[2] < min[imu_1][z_1]){
			min[imu_1][z_1] = mag_raw[2];
		}
		now = std::chrono::system_clock::now();
		elapsed_time = now - start_time; 
	}
	std::cout << "Rotation around x axis is done! Press enter to continue with rotation around y axis" << std::endl;
	std::cin; 
	//Rotate around y axis, find max x and max z
	start_time = std::chrono::system_clock::now();
	while(elapsed_time < 10.0){
		readData();
		//Find x max and x min
		if(mag_raw[0] > max[imu_1][x_1]){
			max[imu_1][x_1] = mag_raw[0];
		}else if(mag_raw[0] < min[imu_1][x_1]){
			min[imu_1][x_1] = mag_raw[0];
		}
		//Find z max and z min
		if(mag_raw[2] > max[imu_1][z_2]){
			max[imu_1][z_2] = mag_raw[2]; 
		}else if(mag_raw[2] < min[imu_1][z_2]){
			min[imu_1][z_2] = mag_raw[2];
		}
		now = std::chrono::system_clock::now();
		elapsed_time = now - start_time; 
	}
	std::cout << "Rotation around y axis is done! Press enter to continue with rotation around z axis" << std::endl; 

	std::cin; 
	//Rotate around z axis, find max/min x and max/min y
	start_time = std::chrono::system_clock::now();
	while(elapsed_time < 10.0){
		readData();
		//Find y max and y min
		if(mag_raw[0] > max[imu_1][x_2]){
			max[imu_1][x_2] = mag_raw[0];
		}else if(mag_raw[0] < min[imu_1][x_2]){
			min[imu_1][x_2] = mag_raw[0];
		}
		//Find x max and d min
		if(mag_raw[1] > max[imu_1][y_2]){
			max[imu_1][y_2] = mag_raw[1]; 
		}else if(mag_raw[1] < min[imu_1][y_2]){
			min[imu_1][y_2] = mag_raw[1];
		}

		now = std::chrono::system_clock::now();
		elapsed_time = now - start_time; 
	}
	std::cout << "Rotation around z is done! Calibration is complete" << std::endl; 

	for (int i = 0; i < 2; i++){ // Two is the number of imu's
		mag_offset[0+i] = (max[imu_1][x_1] + max[imu_1][x_2] + min[imu_1][x_1] + min[imu_1][x_2])/4;
		mag_offset[1+i] = (max[imu_1][y_1] + max[imu_1][y_2] + min[imu_1][y_1] + min[imu_1][y_2])/4;
		mag_offset[2+i] = (max[imu_1][z_1] + max[imu_1][z_2] + min[imu_1][z_1] + min[imu_1][z_2])/4;
	}
}

void MPU9250::timeTest(){

	// Per rotation x,y,z we get two max values and two min values pr imu

	static auto start = std::chrono::system_clock::now(); 
	std::cout << " " << std::endl; 

	auto end = std::chrono::system_clock::now();


	std::chrono::duration<double> elapsed_seconds = end - start; 

	std::cout << "Time elapsed is: " << elapsed_seconds << std::endl; 
}*/