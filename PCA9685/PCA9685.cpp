

#include "PCA9685.h"
#include <wiringPiI2C.h>
#include <stdint.h>
#include <iostream>

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
		// Invalid percentage
		return; 
	}
	if(channel < 0 || channel > 15){
		//Nonexisting channel
		return; 
	}

	if(percentage == 100.0){
		wiringPiI2CWriteReg8(fd,LED0_OFF_H+4*channel, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_OFF_H+4*channel, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_ON_H+4*channel, LED_FULL_ON);
	}else if(percentage == 0){
		wiringPiI2CWriteReg8(fd,LED0_ON_H+4*channel, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_ON_H+4*channel, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_OFF_H+4*channel, LED_FULL_OFF);
	}else{
		uint16_t value = 40.95 * percentage; 
		wiringPiI2CWriteReg8(fd,LED0_ON_H+4*channel, 0x00);
		wiringPiI2CWriteReg8(fd,LED0_ON_L+4*channel, 0x00);

		wiringPiI2CWriteReg8(fd,LED0_OFF_L+4*channel, value); 
		wiringPiI2CWriteReg8(fd,LED0_OFF_H+4*channel, value>>8); // Shift to get the the upper 
	}
}

void PCA9685::setServo(int channel, float percentage){
	//Since the freq is = 50 Hz and the values goes from 0 to 4095 and the servo work between has a dutycycle between 1ms to 2ms 
	//with a period of 20 ms the values should be from 205 to 410 yielding 105 steps
	
	uint16_t value = 0; 
	if(percentage == 0){
		value = 205;
	}else if(percentage == 100){
		value = 410; 
	}else{
		value = 205 + 2.05*percentage; 
	}
	std::cout << "Value is: \t" << value << std::endl; 
	wiringPiI2CWriteReg8(fd,LED0_ON_H+4*channel, 0x00);
	wiringPiI2CWriteReg8(fd,LED0_ON_L+4*channel, 0x00);

	wiringPiI2CWriteReg8(fd,LED0_OFF_L+4*channel, value); 
	wiringPiI2CWriteReg8(fd,LED0_OFF_H+4*channel, value>>8);

}