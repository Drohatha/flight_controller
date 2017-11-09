#include <wiringPiI2C.h>
#include <iostream>
#include "rasp_mpu6050.h"
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <ctime>
#include <chrono>


#define mpuAddress1 0x68
#define mpuAddress2 0x69

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
	accOffset[] = {0,0,0,0,0,0};
	gyroOffset[] = {0,0,0,0,0,0};



	calculatedOffset = false;
	firstIteration = true; 

	roll_gyro = 0; 
	pitch_gyro = 0; 
	yaw_gyro = 0; 

}

void MPU6050::init(){

	// Creates file handles for the I2C 
	fd1 = wiringPiI2CSetup(mpuAddress1);
	fd2 = wiringPiI2CSetup(mpuAddress2); 

	wiringPiI2CWriteReg8(fd1,powerConfig,powerCommand); // set power on
	wiringPiI2CWriteReg8(fd1,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd1, gyroConfig,gyroCommand); // set gyro scale
	wiringPiI2CWriteReg8(fd2,powerConfig,powerCommand); // set power on
	wiringPiI2CWriteReg8(fd2,accelConfig,accelCommand); // set acc scale
	wiringPiI2CWriteReg8(fd2, gyroConfig,gyroCommand); // set gyro scale

	calculateOffset(); 
	std::cout << "Init complete" << std::endl; 
};

void MPU6050::read_data(){

	// Acc in x dir
	accBuffer1[0] = wiringPiI2CReadReg8(fd1, ACCEL_X_H);
	accBuffer1[1] = wiringPiI2CReadReg8(fd1, ACCEL_X_L);
	accBuffer2[0] = wiringPiI2CReadReg8(fd2, ACCEL_X_H);
	accBuffer2[1] = wiringPiI2CReadReg8(fd2, ACCEL_X_L);
	//Acc in y dir
	accBuffer1[2] = wiringPiI2CReadReg8(fd1, ACCEL_Y_H);
	accBuffer1[3] = wiringPiI2CReadReg8(fd1, ACCEL_Y_L);
	accBuffer2[2] = wiringPiI2CReadReg8(fd2, ACCEL_Y_H);
	accBuffer2[3] = wiringPiI2CReadReg8(fd2, ACCEL_Y_L);
	//Acc in z dir
	accBuffer1[4] = wiringPiI2CReadReg8(fd1, ACCEL_Z_H);
	accBuffer1[5] = wiringPiI2CReadReg8(fd1, ACCEL_Z_L);
	accBuffer2[4] = wiringPiI2CReadReg8(fd2, ACCEL_Z_H);
	accBuffer2[5] = wiringPiI2CReadReg8(fd2, ACCEL_Z_L);
	//Gyro around x axis
	gyroBuffer1[0] = wiringPiI2CReadReg8(fd1, GYRO_X_H);
	gyroBuffer1[1] = wiringPiI2CReadReg8(fd1, GYRO_X_L);
	gyroBuffer2[0] = wiringPiI2CReadReg8(fd2, GYRO_X_H);
	gyroBuffer2[1] = wiringPiI2CReadReg8(fd2, GYRO_X_L);
	//GYro around y axis
	gyroBuffer1[2] = wiringPiI2CReadReg8(fd1, GYRO_Y_H);
	gyroBuffer1[3] = wiringPiI2CReadReg8(fd1, GYRO_Y_L);
	gyroBuffer2[2] = wiringPiI2CReadReg8(fd2, GYRO_Y_H);
	gyroBuffer2[3] = wiringPiI2CReadReg8(fd2, GYRO_Y_L);
	//Gyro around z axis
	gyroBuffer1[4] = wiringPiI2CReadReg8(fd1, GYRO_Z_H);
	gyroBuffer1[5] = wiringPiI2CReadReg8(fd1, GYRO_Z_L);
	gyroBuffer2[4] = wiringPiI2CReadReg8(fd2, GYRO_Z_H);
	gyroBuffer2[5] = wiringPiI2CReadReg8(fd2, GYRO_Z_L);


	//Merge H and L acc data for MPU 1
	temp[0] = accBuffer1[0] <<8 | accBuffer1[1];
	temp[1] = accBuffer1[2] <<8 | accBuffer1[3];
	temp[2] = accBuffer1[4] <<8 | accBuffer1[5];
	//Merge H and L gyro data
	temp[3] = gyroBuffer1[0]<<8 | gyroBuffer1[1];
	temp[4] = gyroBuffer1[2]<<8 | gyroBuffer1[3];
	temp[5] = gyroBuffer1[4]<<8 | gyroBuffer1[5];

	//Merge H and L acc data for MPU 2
	temp[6] = accBuffer2[0] <<8 | accBuffer2[1];
	temp[7] = accBuffer2[2] <<8 | accBuffer2[3];
	temp[8] = accBuffer2[4] <<8 | accBuffer2[5];
	//Merge H and L gyro data
	temp[9] = gyroBuffer2[0]<<8 | gyroBuffer2[1];
	temp[10] = gyroBuffer2[2]<<8 | gyroBuffer2[3];
	temp[11] = gyroBuffer2[4]<<8 | gyroBuffer2[5];

	// Convert to m/s^2 for mpu 1
	accXYZ[0] = (temp[0])*gravityAcc/accScale;
	accXYZ[1] = (temp[1])*gravityAcc/accScale;
	accXYZ[2] = (temp[2])*gravityAcc/accScale;
	// Convert to m/s^2 for mpu 2
	accXYZ[3] = (temp[6])*gravityAcc/accScale;
	accXYZ[4] = (temp[7])*gravityAcc/accScale;
	accXYZ[5] = (temp[8])*gravityAcc/accScale;
	// Convert to degrees/s for mpu 1
	gyroXYZ[0] = temp[3]*250/angularScale;
	gyroXYZ[1] = temp[4]*250/angularScale;
	gyroXYZ[2] = temp[5]*250/angularScale;
	// Convert to degrees/s for mpu 2
	gyroXYZ[3] = temp[9]*250/angularScale;
	gyroXYZ[4] = temp[10]*250/angularScale;
	gyroXYZ[5] = temp[11]*250/angularScale;


	//std::cout << "Acc X dir: " << accXYZ[0] << "\tAcc Y dir: " << accXYZ[1] << "\tAcc Z dir: " << accXYZ[2] << std::endl;
	if(calculatedOffset){

		//Add offSet to  acc measurement
		accXYZ[0] -= accOffset[0];
		accXYZ[1] -= accOffset[1];
		accXYZ[2] -= accOffset[2];
		accXYZ[3] -= accOffset[3];
		accXYZ[4] -= accOffset[4];
		accXYZ[5] -= accOffset[5];

		// Add offSet to gyro measurement
		gyroXYZ[0] -= gyroOffset[0];
		gyroXYZ[1] -= gyroOffset[1];
		gyroXYZ[2] -= gyroOffset[2];
		gyroXYZ[3] -= gyroOffset[3];
		gyroXYZ[4] -= gyroOffset[4];
		gyroXYZ[5] -= gyroOffset[5];


		//Merge the measurements from the two different MPU6050 

		mergedAcc[0] = (accXYZ[0] + accXYZ[3])/2.0; 
		mergedAcc[1] = (accXYZ[1] + accXYZ[4])/2.0; 
		mergedAcc[2] = (accXYZ[2] + accXYZ[5])/2.0; 

		mergedGyro[0] = (gyroXYZ[0] + gyroXYZ[3])/2.0; 
		mergedGyro[1] = (gyroXYZ[1] + gyroXYZ[4])/2.0; 
		mergedGyro[2] = (gyroXYZ[2] + gyroXYZ[5])/2.0;


		//calculateEulerAngles(); 
		
	} 
}

void MPU6050::calculateOffset(){
	int n = 1000; 
	for (int i = 0; i < n; ++i){
		read_data();
		accOffset[0] += accXYZ[0];
		accOffset[1] += accXYZ[1];
		accOffset[2] += accXYZ[2] - gravityAcc;
		accOffset[3] += accXYZ[3];
		accOffset[4] += accXYZ[4];
		accOffset[5] += accXYZ[5] - gravityAcc;

		gyroOffset[0] += gyroXYZ[0];
		gyroOffset[1] += gyroXYZ[1];
		gyroOffset[2] += gyroXYZ[2];
		gyroOffset[3] += gyroXYZ[3];
		gyroOffset[4] += gyroXYZ[4];
		gyroOffset[5] += gyroXYZ[5];

	}
	for int(int j = 0; j < 6; j++){
		accOffset[j] = accOffset[j]/n; 
	}

//	std::cout << " Offset x: " << accOffsetX << " Offset y: " << accOffsetY << " Offset z:_" << accOffsetZ << std::endl;
//	std::cout << " Offset x: " << gyroOffsetX << " Offset y: " << gyroOffsetY << " Offset z:_" << gyroOffsetZ << std::endl;
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