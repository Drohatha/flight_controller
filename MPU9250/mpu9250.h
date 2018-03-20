#ifndef MPU9250_H_
#define MPU9250_H_


#include <stdint.h>

class MPU9250{


public: 

	MPU9250(); 
	void init(); 
	void readData(); 
	void calculateOffset(); 
	void calculateEulerAngles();
	void calibrateMagnetometer();

private:
	bool calculated_offset;
	bool first_iteration;  


	int fd_1; 
	int fd_2; 
	int fd_3; 

	int acc_data_buffer[2][6]; 
	int gyro_data_buffer[2][6];
	int mag_data_buffer[2][6]; 
	int16_t acc_merge_buffer[2][3];
	int16_t gyro_merge_buffer[2][3];  
	int16_t mag_merge_buffer[2][3];

	double acc_raw[2][3]; // 2 is Number of imu's, 3 is number of axis
	double gyro_raw[2][3];
	double mag_raw[2][3]; 

	double acc_offset[2][3];
	double gyro_offset[2][3]; 
	double mag_offset[2][3]; 
};

#endif /* RASP_MPU9250_H */