#include "mpu9250.h"


int main(){

 	MPU9250 imu;   
	while(true){
		mpu.readData();
	}
	return 0;
}