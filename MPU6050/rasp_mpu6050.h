#ifndef RASP_MPU6050_H_
#define RASP_MPU6050_H_

#include <stdint.h>
#include <ctime>
#include <chrono>


class MPU6050{



public: 
	MPU6050(int mpu_address);
	void init();
	void read_data(); // This will just spin in loop and update the values we want

private:

	void calculateEulerAngles(); 

	void calculateOffset(); 


	int mpuAddress; 
	bool calculatedOffset; 
	bool firstIteration; 
	float accOffsetX;
	float accOffsetY;
	float accOffsetZ;
	float gyroOffsetX;
	float gyroOffsetY;
	float gyroOffsetZ; 

	int fd; 

	int accOffset[3]; 
	int gyroOffset[3];

	int accBuffer[6];
	int gyroBuffer[6];

	int16_t temp[6];

	float accXYZ[3]; 
	double gyroXYZ[3]; 

	float roll_acc,pitch_acc,roll_gyro,pitch_gyro, yaw_gyro, roll, pitch, yaw; 

	std::chrono::system_clock::time_point t_last;
	std::chrono::system_clock::time_point t_now; 




};



#endif /* RASP_MPU6050_H */