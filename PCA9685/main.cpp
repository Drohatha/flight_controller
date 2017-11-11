#include "PCA9685.h"


int main(){

 	PCA9685 pca; 
	pca.init();
	int percentage = 0.0;   
	while(1){

		
		std::cout << "Write in a value between 0 and 100! "
		std::cin >> percentage; 

		pca.setDutyCycle(0,percentage);

	}	
	return 0;
}