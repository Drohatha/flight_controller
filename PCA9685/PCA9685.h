#ifndef PCA9685_H_
#define PCA9685_H_


class PCA9685{

public: 
	void init(); 
	void setDutyCycle(int channel, float percentage); 
	void setServo(int channel, float percentage); 
	void setCustom(int channel, int number); 
private: 

	int fd;
};








#endif /* RASP_MPU6050_H */
