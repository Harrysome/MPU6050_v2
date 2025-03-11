#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

struct MPU6050{
	void begin(int addl);
	void read();
	float Xaxis();  //return DEG num from 180 ~ -180
	float Yaxis();
	float accX();  //return the accel num
	float accY();
	float accZ();
};

#endif
