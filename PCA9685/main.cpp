#include "PCA9685.h"
#include <iostream>


int main(){

 	PCA9685 pca; 
	pca.init();
	float percentage = 0.0;   
	while(1){

		
		std::cout << "Write in a value between 0 and 100! " << std::endl; 
		std::cin>>percentage; 

		pca.setServo(0,percentage);
		pca.setServo(1,percentage); 

	}	
	return 0;
}