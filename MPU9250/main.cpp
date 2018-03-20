#include "mpu9250.h"


int main(){
 	MPU9250 imu;   
	while(true){
		imu.readData();
		std::cout << "One loop" << std::endl; 
	}
	return 0;
}