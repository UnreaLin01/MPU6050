/*
   This code is refer to the tutorial
   By Dejan https://howtomechatronics.com
*/
#include <Wire.h>
const int IMU = 0x68; // MPU6050 I2C address

// Accelerometer variable
double accX = 0;
double accY = 0;
double accZ = 0;
double accAngleX = 0;
double accAngleY = 0;

// Gryoscope variable
double gyroX = 0;
double gyroY = 0;
double gyroZ = 0;
double gyroAngleX = 0;
double gyroAngleY = 0;
double gyroAngleZ = 0;

//Caculate final angle
double accurateAngleX = 0;
double accurateAngleY = 0;
double accurateAngleZ = 0;

double currentTime = 0;
double previousTime = 0;
double elapsedTime = 0;

//Sensor Compensation Variable
const double accCompX = 0.04204;
const double accCompY = -0.00917;
const double accCompZ = 0.03583;
const double gyroCompX = -4.81954;
const double gyroCompY = -0.40160;
const double gyroCompZ = -2.47504;

void setup() {
  Serial.begin(19200);
  Wire.begin();
  Wire.beginTransmission(IMU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  calibrateIMU();
}
void loop() {
  Wire.beginTransmission(IMU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  accX = ((Wire.read() << 8 | Wire.read()) / 16384.0) - accCompX;
  accY = ((Wire.read() << 8 | Wire.read()) / 16384.0) - accCompY;
  accZ = ((Wire.read() << 8 | Wire.read()) / 16384.0) - accCompZ;

  // Use gravity vector to calculate angle
  // https://www.digikey.com/en/articles/using-an-accelerometer-for-inclination-sensing 
  accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI);
  accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI);

  
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  
  Wire.beginTransmission(IMU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  gyroX = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyroCompX; // deg/sec
  gyroY = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyroCompY; // deg/sec
  gyroZ = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyroCompZ; // deg/sec

  gyroAngleX = gyroAngleX + gyroX * elapsedTime;
  gyroAngleY = gyroAngleY + gyroY * elapsedTime;
  gyroAngleZ = gyroAngleZ + gyroZ * elapsedTime;

  // Complementary filter
  // http://www.pieter-jan.com/node/11
  accurateAngleX = 0.8 * gyroAngleX + 0.2 * accAngleX;
  accurateAngleY = 0.8 * gyroAngleY + 0.2 * accAngleY;
  accurateAngleZ = gyroAngleZ;

  Serial.print("accurateAngleX:");
  Serial.print(accurateAngleX);
  Serial.print(",");
  Serial.print("accurateAngleY:");
  Serial.print(accurateAngleY);
  Serial.print(",");
  Serial.print("accurateAngleZ:");
  Serial.println(accurateAngleZ);
}

void calibrateIMU() {
  double accX = 0;
  double accY = 0;
  double accZ = 0;
  double accErrorX = 0;
  double accErrorY = 0;
  double accErrorZ = 0;
  byte times = 0;
  while (times < 100) {
    Wire.beginTransmission(IMU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU, 6, true);
    accX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    accY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    accZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    accErrorX += accX;
    accErrorY += accY;
    accErrorZ += accZ;
    times++;
  }
  accErrorX = (accErrorX / 100);
  accErrorY = (accErrorY / 100);
  accErrorZ = (accErrorZ / 100) - 1;

  double gyroX = 0;
  double gyroY = 0;
  double gyroZ = 0;
  double gyroErrorX = 0;
  double gyroErrorY = 0;
  double gyroErrorZ = 0;
  times = 0;
  while (times < 100) {
    Wire.beginTransmission(IMU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU, 6, true);
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    gyroErrorX = gyroErrorX + (gyroX / 131.0);
    gyroErrorY = gyroErrorY + (gyroY / 131.0);
    gyroErrorZ = gyroErrorZ + (gyroZ / 131.0);
    times++;
  }
  gyroErrorX = gyroErrorX / 100;
  gyroErrorY = gyroErrorY / 100;
  gyroErrorZ = gyroErrorZ / 100;

  Serial.print("accelerometerErrorX:");
  Serial.println(accErrorX, 5);
  Serial.print("accelerometerErrorY:");
  Serial.println(accErrorY, 5);
  Serial.print("accelerometerErrorZ:");
  Serial.println(accErrorZ, 5);

  Serial.print("gyroErrorX:");
  Serial.println(gyroErrorX, 5);
  Serial.print("gyroErrorY:");
  Serial.println(gyroErrorY, 5);
  Serial.print("gyroErrorZ:");
  Serial.println(gyroErrorZ, 5);
}
