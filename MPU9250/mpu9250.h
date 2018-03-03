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

private:
	bool calculated_offset;
	bool first_iteration;  


	int fd_1; 
	int fd_2; 
	int fd_3; 

	int acc_data_buffer[12]; 
	int gyro_data_buffer[12];
	int mag_data_buffer[12]; 
	int16_t acc_merge_buffer[6];
	int16_t gyro_merge_buffer[6];  
	int16_t mag_merge_buffer[6];

	float acc_raw[6]; 
	float gyro_raw[6];

	float acc_offset[6];
	float gyro_offset[6]; 

};

#endif /* RASP_MPU9250_H */