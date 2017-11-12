#include "PCA9685.h"
#include <iostream>


int main(){

 	PCA9685 pca; 
	pca.init();
	float percentage = 0.0;   
	while(1){

		
		std::cout << "Write in a value between 0 and 100! " << std::endl; 
		std::cin>>percentage; 

		pca.setCustom(0,percentage);
		pca.setCustom(1,percentage); 
		pca.setCustom(2,percentage);
		pca.setCustom(3,percentage);

	}	
	return 0;
}