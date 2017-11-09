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

	bool calculatedOffset; 
	bool firstIteration; 


	float accOffset[6]; // XYZ 1, XYZ 2 
	float gyroOffset[3]; // XYZ 1, XYZ 2

	int fd1,fd2; 

	int accBuffer1[6]; 
	int accBuffer2[6]; 

	int gyroBuffer1[1]; 
	int gyroBuffer2[2];

	/*float accOffsetX;
	float accOffsetY;
	float accOffsetZ;
	float gyroOffsetX;
	float gyroOffsetY;
	float gyroOffsetZ; 



	int accOffset[3]; 
	int gyroOffset[3];
*/

	int16_t temp[12];

	float accXYZ[6]; 
	float gyroXYZ[6];

	float mergedAcc[3]; 
	float mergedGyro[3]; 


	float roll_acc,pitch_acc,roll_gyro,pitch_gyro, yaw_gyro, roll, pitch, yaw; 

	std::chrono::system_clock::time_point t_last;
	std::chrono::system_clock::time_point t_now; 




};



#endif /* RASP_MPU6050_H */