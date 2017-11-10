

#include "mpu6050.h"


int main(){


/*
	
	int acc_x_high = 0; // 8 bit higher acc data
	int acc_x_low = 0; // 8 bit lower acc data
	int16_t acc_16_bit = 0; // The data from the MPU6050 is 16 bits
	float acc = 0.0; 


	int fd, result; 
	fd = wiringPiI2CSetup(0x68);
	std::cout << "Init result: "<< fd << std::endl;
	mpu6050_init(fd);


	while(1){
		acc_x_high = wiringPiI2CReadReg8(fd, ACCEL_Z_H);
		acc_x_low = wiringPiI2CReadReg8(fd, ACCEL_Z_L);

		acc_16_bit = acc_x_high <<8 | acc_x_low;

		acc = (acc_16_bit*gravityAcc)/accScale;
		std::cout << "Acc_y_h: " << acc_x_high << std::endl; 
		std::cout << "Acc_y_l: " << acc_x_low << std::endl; 
		std::cout << "ACC i z-akse: " << acc << std::endl; 

	}

	*/
 	MPU6050 mpu; 
	mpu.init();  
	while(1){

		mpu.read_data();

	}
		

    
	return 0;
}