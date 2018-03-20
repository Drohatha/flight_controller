#include "mpu9250.h"
#include <iostream>


int main(){
 	MPU9250 imu;   
	while(true){
		imu.readData();
		std::cout << "One loop" << std::endl; 
	}
	return 0;
}