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
#define magCommand 0b00000110 //Continuous measurement mode 2, poor documentation of what this actually is 

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

enum acceleration {
	acc_x_mpu_1,
	acc_x_mpu_2,
	acc_y_mpu_1,
	acc_y_mpu_2,
	acc_z_mpu_1,
	acc_z_mpu_2
}; 

enum gyro{
	gyro_x_mpu_1,
	gyro_x_mpu_2,
	gyro_y_mpu_1,
	gyro_y_mpu_2,
	gyro_z_mpu_1,
	gyro_z_mpu_2
};

enum mag{
	mag_x_mpu_1,
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
	wiringPiI2CWriteReg8(fd_3,magModeConfig,magCommand); 


	calculateOffset(); 
}

void MPU9250::readData(){

	//Vurder om man skal skrive det her til en for-løkke istedenfor

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

	mag_data_buffer[0] = wiringPiI2CReadReg8(fd_3,MAG_X_H);
	mag_data_buffer[1] = wiringPiI2CReadReg8(fd_3,MAG_X_L);

	mag_data_buffer[2] = wiringPiI2CReadReg8(fd_3,MAG_Y_H);
	mag_data_buffer[3] = wiringPiI2CReadReg8(fd_3,MAG_Y_L);

	mag_data_buffer[4] = wiringPiI2CReadReg8(fd_3,MAG_Z_H);
	mag_data_buffer[5] = wiringPiI2CReadReg8(fd_3,MAG_Z_L);
	wiringPiI2CReadReg8(fd_3, MAG_STATUS); 

	for (int i = 0; i < 6; ++i){
		std::cout<< mag_data_buffer[i] << std::endl; 
		/* code */
	}
	std::cout << "One read" << std::endl; 
	/*
	for (int i = 0; i < 12; ++i)
	{	
			std::cout << "Error occured: " << acc_data_buffer[i] << std::endl; 

			std::cout << "Error occured: " << gyro_data_buffer[i] << std::endl; 

		
	}*/

	//Acc
	acc_merge_buffer[acc_x_mpu_1] = acc_data_buffer[0] << 8| acc_data_buffer[1]; // X  from mpu 1
	acc_merge_buffer[acc_x_mpu_2] = acc_data_buffer[2] << 8|  acc_data_buffer[3]; // X from mpu 2

	acc_merge_buffer[acc_y_mpu_1] = acc_data_buffer[4] << 8| acc_data_buffer[5]; // Y from mpu 1
	acc_merge_buffer[acc_y_mpu_2] = acc_data_buffer[6] << 8| acc_data_buffer[7]; // Y from mpu 2

	acc_merge_buffer[acc_z_mpu_1] = acc_data_buffer[8] << 8| acc_data_buffer[9]; // Z from mpu 1
	acc_merge_buffer[acc_z_mpu_2] = acc_data_buffer[10] << 8| acc_data_buffer[11]; // Z from mpu 2
	//Gyro
	gyro_merge_buffer[gyro_x_mpu_1] = gyro_data_buffer[0] << 8| gyro_data_buffer[1]; // X from mpu 1
	gyro_merge_buffer[gyro_x_mpu_2] = gyro_data_buffer[2] << 8| gyro_data_buffer[3]; // X from mpu 2

	gyro_merge_buffer[gyro_y_mpu_1] = gyro_data_buffer[4] << 8| gyro_data_buffer[5]; // Y from mpu 1
	gyro_merge_buffer[gyro_y_mpu_2] = gyro_data_buffer[6] << 8| gyro_data_buffer[7]; // Y from mpu 2

	gyro_merge_buffer[gyro_z_mpu_1] = gyro_data_buffer[8] << 8| gyro_data_buffer[9]; // Z from mpu 1
	gyro_merge_buffer[gyro_z_mpu_2] = gyro_data_buffer[10] << 8| gyro_data_buffer[11]; // Z from mpu 2

	//Mag 
	mag_merge_buffer[mag_x_mpu_1] = mag_data_buffer[0] << 8| mag_data_buffer[1];
	mag_merge_buffer[mag_x_mpu_1] = mag_data_buffer[2] << 8| mag_data_buffer[3];
	mag_merge_buffer[mag_x_mpu_1] = mag_data_buffer[4] << 8| mag_data_buffer[5];

	/*
	std::cout << "Mag x read: " <<mag_merge_buffer[mag_x_mpu_1] 
	<< "Mag y read: " << mag_merge_buffer[mag_y_mpu_1] << "Mag z read: " << mag_merge_buffer[mag_z_mpu_1]
	<< std::endl;
*/
	//Convert to m/s^2
	//X axis 
	acc_raw[acc_x_mpu_1] = acc_merge_buffer[acc_x_mpu_1]*gravity/accScale;
	acc_raw[acc_x_mpu_2] = acc_merge_buffer[acc_x_mpu_2]*gravity/accScale;
	//Y axis MPU 1 & 2
	acc_raw[acc_y_mpu_1] = acc_merge_buffer[acc_y_mpu_1]*gravity/accScale;
	acc_raw[acc_y_mpu_2] = acc_merge_buffer[acc_y_mpu_2]*gravity/accScale;
	//Z axis MPU 1 & 2
	acc_raw[acc_z_mpu_1] = acc_merge_buffer[acc_z_mpu_1]*gravity/accScale;
	acc_raw[acc_z_mpu_2] = acc_merge_buffer[acc_z_mpu_2]*gravity/accScale;

	//Convert to degrees/s
	gyro_raw[gyro_x_mpu_1] = gyro_merge_buffer[gyro_x_mpu_1]*250/angularScale; // 250 deg/s is max measurement output
	gyro_raw[gyro_x_mpu_2] = gyro_merge_buffer[gyro_x_mpu_2]*250/angularScale; 

	gyro_raw[gyro_y_mpu_1] = gyro_merge_buffer[gyro_y_mpu_1]*250/angularScale; 
	gyro_raw[gyro_y_mpu_2] = gyro_merge_buffer[gyro_y_mpu_2]*250/angularScale; 

	gyro_raw[gyro_z_mpu_1] = gyro_merge_buffer[gyro_z_mpu_1]*250/angularScale; 
	gyro_raw[gyro_z_mpu_2] = gyro_merge_buffer[gyro_z_mpu_2]*250/angularScale; 

	//mag_raw[mag_x_mpu_1] = mag_merge_buffer[mag_x_mpu_1]*
	//mag_raw[mag_y_mpu_1] = mag_merge_buffer[mag_y_mpu_1]*
	//mag_raw[mag_z_mpu_1] = mag_merge_buffer[mag_z_mpu_1]*

	if(calculated_offset){
		for (int i = 0; i < 6; ++i){
			acc_raw[i] -= acc_offset[i];
			gyro_raw[i] -= gyro_offset[i];
		}
		//std::cout << "Z from mpu 1 " << acc_raw[acc_z_mpu_1] << "\t Z from mpu 2 " << acc_raw[acc_z_mpu_2] << std::endl;
		//std::cout << "Z gyro mpu 1 " << gyro_raw[gyro_z_mpu_1] << "\t Z gyro mpu 2 " << gyro_raw[gyro_z_mpu_2] << std::endl;  
	}
}

void MPU9250::calculateOffset(){
	int n = 1000; 
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

	calculated_offset = true; 
}