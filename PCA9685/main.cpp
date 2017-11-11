#include "PCA9685.h"


int main(){

 	PCA9685 pca; 
	pca.init();  
	while(1){
		for (int i = 50; i < 100; ++i)
		{
			pca.setDutyCycle(0, 50); 
		}
		

	}
		
	return 0;
}