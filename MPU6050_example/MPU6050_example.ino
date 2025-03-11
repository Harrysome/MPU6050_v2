#include <MPU6050.h>

MPU6050 mpu;
const int addr = 0x68;   //the addr of MPU6050
const int sen = 0;   //the sensitivity from 0 ~ 6

void setup() {///
  Serial.begin(9600);
  mpu.begin(addr);
}

void loop() {
  mpu.read();   //read all the num

  Serial.print("mpu Xaxis = ");
  Serial.println(mpu.Xaxis());
  Serial.print("mpu Yaxis = ");
  Serial.println(mpu.Yaxis());
  Serial.println("---------------------------------------------");
  delay(250);
  
  /*
  Serial.print("mpu accX = ");
  Serial.println(mpu.accX());
  Serial.print("mpu accY = ");
  Serial.println(mpu.accY());
  Serial.print("mpu accZ = ");
  Serial.println(mpu.accZ());
  Serial.println("---------------------------------------------");
  delay(250);
  */
}
