#include <MPU6050.h>
#include <Wire.h>

int addr;

float pi = 3.1415926;
float accRate = 16384.0;
float gRate = 131.0;
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; // Raw data
float accx, accy, accz;
float x, y;      // Angle
float xoffset = 0, yoffset = 0;

// Kalman filter variables
float kalX = 0, kalY = 0;
float P_x[2][2] = {{1, 0}, {0, 1}}; // Covariance matrix for X
float P_y[2][2] = {{1, 0}, {0, 1}}; // Covariance matrix for Y
float Q_angle = 0.003, Q_bias = 0.005, R_measure = 0.03; // Noise parameters
float biasX = 0, biasY = 0;
unsigned long lastTime;

void MPU6050::begin(int add) {
    addr = add;

    Wire.begin();
    Wire.beginTransmission(addr);   // Sampling rate
    Wire.write(0x19);
    Wire.write(0x07);
    Wire.endTransmission(true);

    Wire.begin();
    Wire.beginTransmission(addr);   // Clock select
    Wire.write(0x6B);
    Wire.write(0x01);
    Wire.endTransmission(true);

    Wire.begin();
    Wire.beginTransmission(addr);
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

    lastTime = millis();
}

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

    accx = ax / accRate;
    accy = ay / accRate;
    accz = az / accRate;

    x = (atan(accy / accz) * 180.0 / pi) - xoffset; // Compute tilt angles
    y = (atan(accx / accz) * 180.0 / pi) - yoffset;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // Time interval
    lastTime = now;

    // Apply Kalman Filter for X-axis
    float rateX = gx / gRate - biasX;
    kalX += dt * rateX;
    P_x[0][0] += dt * (dt * P_x[1][1] - P_x[0][1] - P_x[1][0] + Q_angle);
    P_x[0][1] -= dt * P_x[1][1];
    P_x[1][0] -= dt * P_x[1][1];
    P_x[1][1] += Q_bias * dt;

    float Sx = P_x[0][0] + R_measure;
    float Kx[2] = {P_x[0][0] / Sx, P_x[1][0] / Sx};

    float yx = x - kalX;
    kalX += Kx[0] * yx;
    biasX += Kx[1] * yx;

    P_x[0][0] -= Kx[0] * P_x[0][0];
    P_x[0][1] -= Kx[0] * P_x[0][1];
    P_x[1][0] -= Kx[1] * P_x[0][0];
    P_x[1][1] -= Kx[1] * P_x[0][1];

    // Apply Kalman Filter for Y-axis
    float rateY = gy / gRate - biasY;
    kalY += dt * rateY;
    P_y[0][0] += dt * (dt * P_y[1][1] - P_y[0][1] - P_y[1][0] + Q_angle);
    P_y[0][1] -= dt * P_y[1][1];
    P_y[1][0] -= dt * P_y[1][1];
    P_y[1][1] += Q_bias * dt;

    float Sy = P_y[0][0] + R_measure;
    float Ky[2] = {P_y[0][0] / Sy, P_y[1][0] / Sy};

    float yy = y - kalY;
    kalY += Ky[0] * yy;
    biasY += Ky[1] * yy;

    P_y[0][0] -= Ky[0] * P_y[0][0];
    P_y[0][1] -= Ky[0] * P_y[0][1];
    P_y[1][0] -= Ky[1] * P_y[0][0];
    P_y[1][1] -= Ky[1] * P_y[0][1];
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

float MPU6050::kal_x() {
    return kalX;
}

float MPU6050::kal_y() {
    return kalY;
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
