#ifndef MPU6050_H_
#define MPU6050_H_


#include <stdint.h>

class MPU6050{


public: 

	MPU6050(); 
	void init(); 
	void readData(); 
	void calculateEulerAngles(); 

private:
	bool calculated_offset;
	bool first_iteration;  


	int fd_1; 
	int fd_2; 

	int acc_data_buffer[12]; 
	int gyro_data_buffer[12]; 
	int16_t acc_merge_buffer[6];
	int16_t gyro_merge_buffer[6];  

	float acc_raw[6]; 
	float gyro_raw[6];

	float acc_offset[6];
	float gyro_offset[6]; 

};

#endif /* RASP_MPU6050_H */