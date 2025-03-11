#include <MPU6050.h>
#include <Wire.h>

int addr;

float pi = 3.1415926;
float accRate = 16384.0;
float gRate = 131.0;
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; //raw data
float accx, accy, accz;
float x, y;      //angle
float xoffset = 0, yoffset = 0;

void MPU6050::begin(int add) {
    addr = add;

    Wire.begin();
    Wire.beginTransmission(addr);   //sampling rate
    Wire.write(0x19);
    Wire.write(0x07);
    Wire.endTransmission(true);

    Wire.begin();
    Wire.beginTransmission(addr);   //clock select
    Wire.write(0x6B);
    Wire.write(0x01);
    Wire.endTransmission(true);

    Wire.begin();
    Wire.beginTransmission(addr);   //
    Wire.write(0x6C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(addr);
    Wire.write(0x1A);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(addr);
    Wire.write(0x1B);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(addr);
    Wire.write(0x1C);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(addr);
    Wire.write(0x23);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(addr);
    Wire.write(0x38);
    Wire.write(0x01);
    Wire.endTransmission(true);

    Wire.beginTransmission(addr);
    Wire.write(0x68);
    Wire.write(0);
    Wire.endTransmission(true);

}

// void MPU6050_Base::initialize() {
//     setClockSource(MPU6050_CLOCK_PLL_XGYRO);
//     setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//     setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//     setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
// }



void MPU6050::read() {
    Wire.beginTransmission(addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, 14, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();

    //accx = map(rx,min,max,-90,90); //設定分量在90~-90
    //accy = map(ry,min,max,-90,90); //固定比例
    //accz = map(rz,min,max,-90,90);

    accx = (ax) / accRate;
    accy = (ay) / accRate;
    accz = (az) / accRate;

    x = (atan(accy / accz) * 180.0 / pi) - xoffset; //取tan^-1 -> 角度
    y = (atan(accx / accz) * 180.0 / pi) - yoffset;

    //if(x>180)x -= 360;
    //if(y>180)y -= 360;
    //if(z>180)z -= 360;
}

void MPU6050::calibrate() {
    float x_tmp = 0.0, y_tmp = 0.0;
    for (int i = 0; i < 100; i++) {
        read();
        x_tmp += x;
        y_tmp += y;
    }
    xoffset = x_tmp / 100.0;
    yoffset = y_tmp / 100.0;
}

float MPU6050::Xaxis() {
    return x;
}

float MPU6050::Yaxis() {
    return y;
}

float MPU6050::accX() {
    return accx;
}

float MPU6050::accY() {
    return accy;
}

float MPU6050::accZ() {
    return accz;
}

float MPU6050::x_offset() {
    return xoffset;
}

float MPU6050::y_offset() {
    return yoffset;
}
