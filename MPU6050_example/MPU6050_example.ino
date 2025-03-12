#include <MPU6050.h>

MPU6050 mpu;
const int addr = 0x68;   //the addr of MPU6050

void setup() {///
  Serial.begin(9600);
  mpu.begin(addr);
  mpu.calibrate();
}

void plotData() {
    Serial.print("RawX:"); Serial.print(mpu.Xaxis());
    Serial.print("\tRawY:"); Serial.print(mpu.Yaxis());
    Serial.print("\tKalX:"); Serial.print(mpu.kal_x());
    Serial.print("\tKalY:"); Serial.println(mpu.kal_y());
}

void loop() {
  mpu.read();   //read all the num

  // Serial.print("mpu Xaxis = ");
  // Serial.println(mpu.Xaxis());
  // Serial.print("mpu Yaxis = ");
  // Serial.println(mpu.Yaxis());
  // Serial.println("---------------------------------------------");
  // delay(250);

  // Serial.print("x offset = ");
  // Serial.print(mpu.x_offset());
  // Serial.print(", y offset = ");
  // Serial.println(mpu.y_offset());
  // Serial.println("---------------------------------------------");
  // delay(250);
  
  // Serial.print("mpu accX = ");
  // Serial.println(mpu.accX());
  // Serial.print("mpu accY = ");
  // Serial.println(mpu.accY());
  // Serial.print("mpu accZ = ");
  // Serial.println(mpu.accZ());
  // Serial.println("---------------------------------------------");
  // delay(250);

  // Serial.print("mpu kal_X = ");
  // Serial.println(mpu.kal_x());
  // Serial.print("mpu kal_Y = ");
  // Serial.println(mpu.kal_y());
  // Serial.println("---------------------------------------------");

  plotData();
  delay(100);
}
