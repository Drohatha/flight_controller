#ifndef MPU6050_H_
#define MPU6050_H_


class MPU6050{


public: 

	MPU6050(); 
	void init(); 
	void read_data(); 

private:


	int fd_1; 
	int fd_2; 

	int accDataBuffer[12]; 
	int gyroDataBuffer[12]; 
	int accMergeBuffer[6];
	int gyroMergeBuffer[6];  
};

#endif /* RASP_MPU6050_H */