

#include "PCA9685.h"
#include <wiringPiI2C.h>
#include <stdint.h>


//PCA registers
#define PCA_ADDRESS 0x40

#define MODE_1 0x00

#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

#define PRE_SCALE 0xFE


//PCA commands

#define ON 0x00
#define OFF 0x10 

#define LED_FULL_ON 0x10
#define LED_FULL_OFF 0x10


#define PRE_SCALE_VALUE 0x79 // round(osc_clock/(4096*update_rate))-1, = round(25 MHz/(4096*50Hz))-1 = 121 = 0x79

void PCA9685::init(){

	fd = wiringPiI2CSetup(PCA_ADDRESS);
	wiringPiI2CWriteReg8(fd,MODE_1, OFF); // The sleep bit = 1 to change prescale value of the clock
	wiringPiI2CWriteReg8(fd,PRE_SCALE, PRE_SCALE_VALUE); 
	wiringPiI2CWriteReg8(fd,MODE_1, ON); 

}

void PCA9685::setDutyCycle(int channel, float percentage){
	if(percentage > 100.0 || percentage < 0.0){
		return; 
	}

	if(percentage == 100.0){
		wiringPiI2CWriteReg8(fd,LED0_OFF_H, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_OFF_H, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_ON_H, LED_FULL_ON);
	}else if(percentage == 0){
		wiringPiI2CWriteReg8(fd,LED0_ON_H, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_ON_H, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_OFF_H, LED_FULL_OFF);
	}else{
		uint16_t value = 40.95 * percentage; 
		wiringPiI2CWriteReg8(fd,LED0_ON_H, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_ON_L, 0x00);

		wiringPiI2CWriteReg8(fd,LED0_OFF_L, value); 
		wiringPiI2CWriteReg8(fd,LED0_OFF_H, value>>8); // Shift to get the the upper 

	}
}