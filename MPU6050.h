#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>

struct MPU6050 {
	void begin(int add);
	void read();
	void calibrate();
	float Xaxis();  //return DEG num from 180 ~ -180
	float Yaxis();
	float accX();  //return the accel num
	float accY();
	float accZ();
	float x_offset();
	float y_offset();
};

#endif
