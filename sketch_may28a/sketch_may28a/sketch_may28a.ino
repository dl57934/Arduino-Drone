#include <Wire.h>
int MPU = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
float dT;
float finalAccelX, finalAccelY, finalAccelZ;
float finalGyroX, finalGyroY, finalGyroZ;
float complememtX, complememtY, complememtZ;
unsigned long Tnow, Tprev;
void setup()
{
  SetI2c();
  InitDt();
  getInitavg();
  /* add setup code here */
}

void loop()
{
  getAngle();
  calDt();
  calAccel();
  calGyro();
  complementary();
  /* add main program code here */

}
void getAngle()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY=Wire.read() << 8 | Wire.read();
  accelZ=Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6);
  gyroX= Wire.read() << 8 | Wire.read();
  gyroY=Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}
void getInitavg()
{
  float sumAcX, sumAcY, sumAcZ;
  float sumGyX, sumGyY, sumGyZ;
  for (int i = 0; i < 10; i++)
  {
    getAngle();
    sumAcX += accelX; sumAcY += accelY; sumAcZ += accelZ;
    sumGyX += gyroX; sumGyY += gyroY; sumGyZ += gyroZ;
  }
  baseAcX = sumAcX / 10;    baseAcY = sumAcY / 10;    baseAcZ = sumAcZ / 10;
  baseGyX = baseGyX / 10;   baseGyY = baseGyY / 10;   baseGyZ = baseGyZ / 10;
}
void InitDt()
{
  Tprev = micros();
}
void calDt()
{
  Tnow = micros();
  dT = (Tnow - Tprev)/1000000.0;
  Tprev = Tnow;
}
void SetI2c()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
float accel_x, accel_y, accel_z;
float accel_xz, accel_yz;
void calAccel()
{
  
  accel_x = accelX - baseAcX;
  accel_y = accelY - baseAcY;
  accel_z = accelZ - (16384 - baseAcZ);
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  finalAccelY = atan(-accel_x/accel_yz) * 180 / PI;
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  finalAccelX = atan(accel_y / accel_xz) * 180 / PI;
  finalAccelZ = 0;
}
float gyro_x, gyro_y, gyro_z;
float gyro_xz, gyro_yz;
void calGyro()
{
  
  gyro_x=(gyroX - baseGyX) / 131;
  gyro_y=(gyroY - baseGyY) / 131;
  gyro_z=(gyroZ - baseGyZ) / 131;
  finalGyroX += gyro_x*dT;
  finalGyroY += gyro_y*dT;
  finalGyroZ += gyro_z*dT;
}
void complementary()
{
  float tmpX, tmpY, tmpZ;
  tmpX= complememtX+gyro_x*dT;
  tmpY = complememtY+gyro_y*dT;
  tmpZ = complememtZ+gyro_z*dT;
  complememtX = 0.96*tmpX + 0.04*finalAccelX;
  complememtY = 0.96*tmpY + 0.04*finalAccelY;
  complememtZ = tmpZ;
  Serial.print("Roll ");Serial.print(complememtX);Serial.print(" Pitch ");Serial.print(complememtY);
  Serial.print(" Yaw ");Serial.println(complememtZ);
}
