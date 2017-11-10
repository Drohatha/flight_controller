#include <wiringPiI2C.h>
#include <iostream>
#include "rasp_mpu6050.h"
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <ctime>
#include <chrono>


#define gravityAcc 9.81
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

//Define the registers of gyro and accelerometer 

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


MPU6050::MPU6050(){

	accOffsetX = 0;
	accOffsetY = 0;
	accOffsetZ = 0; 
	gyroOffsetX = 0;
	gyroOffsetY = 0;
	gyroOffsetZ = 0;

	calculatedOffset = false;
	firstIteration = true; 

	roll_gyro = 0; 
	pitch_gyro = 0; 
	yaw_gyro = 0; 

}

void MPU6050::init(){

	fd = wiringPiI2CSetup(mpuAddress);
	wiringPiI2CWriteReg8(fd,powerConfig,powerCommand); // set power in
	wiringPiI2CWriteReg8(fd,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd, gyroConfig,gyroCommand); // set gyro scale
	calculateOffset(); 
	std::cout << "Init complete" << std::endl; 
};

void MPU6050::read_data(){

	accBuffer[0] = wiringPiI2CReadReg8(fd, ACCEL_X_H);
	accBuffer[1] = wiringPiI2CReadReg8(fd, ACCEL_X_L);
	accBuffer[2] = wiringPiI2CReadReg8(fd, ACCEL_Y_H);
	accBuffer[3] = wiringPiI2CReadReg8(fd, ACCEL_Y_L);
	accBuffer[4] = wiringPiI2CReadReg8(fd, ACCEL_Z_H);
	accBuffer[5] = wiringPiI2CReadReg8(fd, ACCEL_Z_L);

	gyroBuffer[0] = wiringPiI2CReadReg8(fd, GYRO_X_H);
	gyroBuffer[1] = wiringPiI2CReadReg8(fd, GYRO_X_L);
	gyroBuffer[2] = wiringPiI2CReadReg8(fd, GYRO_Y_H);
	gyroBuffer[3] = wiringPiI2CReadReg8(fd, GYRO_Y_L);
	gyroBuffer[4] = wiringPiI2CReadReg8(fd, GYRO_Z_H);
	gyroBuffer[5] = wiringPiI2CReadReg8(fd, GYRO_Z_L);

	//Merge H and L acc data
	temp[0] = accBuffer[0] <<8 | accBuffer[1];
	temp[1] = accBuffer[2] <<8 | accBuffer[3];
	temp[2] = accBuffer[4] <<8 | accBuffer[5];
	//Merge H and L gyro data
	temp[3] = gyroBuffer[0]<<8 | gyroBuffer[1];
	temp[4] = gyroBuffer[2]<<8 | gyroBuffer[3];
	temp[5] = gyroBuffer[4]<<8 | gyroBuffer[5];

	accXYZ[0] = (temp[0])*gravityAcc/accScale;
	accXYZ[1] = (temp[1])*gravityAcc/accScale;
	accXYZ[2] = (temp[2])*gravityAcc/accScale;

	gyroXYZ[0] = temp[3]*250/angularScale;
	gyroXYZ[1] = temp[4]*250/angularScale;
	gyroXYZ[2] = temp[5]*250/angularScale;
	//std::cout << "Acc X dir: " << accXYZ[0] << "\tAcc Y dir: " << accXYZ[1] << "\tAcc Z dir: " << accXYZ[2] << std::endl;
	if(calculatedOffset){
		accXYZ[0] -= accOffsetX;
		accXYZ[1] -= accOffsetY;
		accXYZ[2] -= accOffsetZ;

		gyroXYZ[0] -= gyroOffsetX;
		gyroXYZ[1] -= gyroOffsetY;
		gyroXYZ[2] -= gyroOffsetZ;
		//std::cout << "Acc X dir: " << accXYZ[0] << "\tAcc Y dir: " << accXYZ[1] << "\tAcc Z dir: " << accXYZ[2] << std::endl;
		/*std::cout << "Gyro X dir: " << gyroXYZ[0] << "\t Gyro Y dir: " << gyroXYZ[1]*/ std::cout<< "Address: " << mpuAddress<< "\t Gyro Z dir: " << gyroXYZ[2] <<std::endl;

		//calculateEulerAngles(); 
		
	} 
}

void MPU6050::calculateOffset(){
	int n = 1000; 
	for (int i = 0; i < n; ++i){
		read_data();
		accOffsetX += accXYZ[0];
		accOffsetY += accXYZ[1];
		accOffsetZ += accXYZ[2] - gravityAcc;
		gyroOffsetX += gyroXYZ[0];
		gyroOffsetY += gyroXYZ[1];
		gyroOffsetZ += gyroXYZ[2];

	}
	accOffsetX = accOffsetX/n;
	accOffsetY = accOffsetY/n;
	accOffsetZ = accOffsetZ/n; 

	gyroOffsetX = gyroOffsetX/n;
	gyroOffsetY = gyroOffsetY/n;
	gyroOffsetZ = gyroOffsetZ/n;


	std::cout << " Offset x: " << accOffsetX << " Offset y: " << accOffsetY << " Offset z:_" << accOffsetZ << std::endl;
	std::cout << " Offset x: " << gyroOffsetX << " Offset y: " << gyroOffsetY << " Offset z:_" << gyroOffsetZ << std::endl;
	calculatedOffset = true; 
}

void MPU6050::calculateEulerAngles(){
	double duration; 
	if(firstIteration){
		t_last = std::chrono::system_clock::now();
		firstIteration = false; 
	}

	double temp = sqrt(pow(accXYZ[1],2)+pow(accXYZ[2],2));
	roll_acc = 180*atan2(accXYZ[1],accXYZ[2])/pi; 
	pitch_acc = 180*atan2(-accXYZ[0],temp)/pi;
	
	//Time calculation 
	t_now = std::chrono::system_clock::now(); 
	std::chrono::duration<double> elapsed_seconds = t_now-t_last;
	duration = elapsed_seconds.count(); 
    t_last = t_now; 

	roll_gyro += duration * gyroXYZ[0]; 
	pitch_gyro += duration * gyroXYZ[1]; 
	yaw_gyro += duration * gyroXYZ[2]; 



	std::cout << "Roll: " << roll_gyro << "\t Pitch: " << pitch_gyro << "\t Yaw "<< yaw_gyro << std::endl; 
	//std::cout << "Roll: " << roll << "\t Pitch: " << pitch << std::endl; 
}





/*
	Hva vi må teste: 

	Kjør begge gyroskopene opp mot hverandre, og se på vinkelhastigheten på begge to! Er disse omtrent like?

	Hvis ja.. ehm.. da må det være noe kodefeil en plass 


	std::clock_t start;
    double duration;

    start = std::clock();

    
    while(1){
    	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    	std::cout<<"Clock: "<< duration <<'\n';
    }*/