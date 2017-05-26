#include <Wire.h>
#define Throttle_Max 255
#define Throttle_Min 0
#define Throttle_InI 5
enum {
  head1, head2, head3,
  dataSize, dataType,
  Roll, Pitch, Yaw, Throtte, Aux,
  crc, PackSize
};
int Throttle;
int MPU_Address = 0x68;
double acX, acY, acZ, gyX, gyY, gyZ;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
float accelAngX, accelAngY, accelAngZ;
float gyroX, gyroY, gyroZ;
float gyroAngX, gyroAngY, gyroAngZ;
float complementX, complementY, complementZ;
unsigned long tNow;
unsigned long tPrev;
unsigned long dT;
int motorA = 5;
int motorB = 6;
int motorC = 9;
int motorD = 10;
uint8_t msp[PackSize];
void initMPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
}
void setup()
{
  initMPU();
  Serial.begin(115200);
  Serial1.begin(115200);
  calibAccelGyro();
  initTime();
}
void loop()
{
  readAccelGyro();
  getDt();
  getAcc();
  getGyro();
  compementary();
  initMotorSpeed();
  static int cnt;
  cnt++;
  if (cnt % 2 == 0)
    SendDataToProcessing();
}
void readAccelGyro()
{
  Wire.beginTransmission(MPU_Address); //마스터 장치에서 슬레이브 장치로 데이터 전송시작한다.
  Wire.write(0x3B); //마스터 장치에서 슬레이브 장치로 데이터 전송한다. 큐에저장한다
  Wire.endTransmission(false);//false면 데이터 전송이끝나도  연결유지한다. 
  Wire.requestFrom(MPU_Address, 6); // 슬레이브 장치가 자이로센서에게 데이터요청한다. 6바이트  
  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(MPU_Address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_Address, 6);
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();
}
void calibAccelGyro()
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  readAccelGyro();
  for (int i = 0; i < 10; i++)
  {
    readAccelGyro();
    sumAcX += acX, sumAcY += acY, sumAcZ += acZ;
    sumGyX += gyX, sumGyY += gyY, sumGyZ += gyZ;
  }
  baseAcX = sumAcX / 10, baseAcY = sumAcY / 10, baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10, baseGyY = sumGyY / 10, baseGyZ = sumGyZ / 10;
}
void initTime()
{
  tPrev = micros();
}
void getDt()
{
  tNow = micros();
  dT = (tNow - tPrev) / 1000000.0;
  tPrev = tNow;
}
void SendDataToProcessing()
{
  Serial.print(F("DEL:"));
  Serial.print(dT, DEC);
  Serial.print(F("#ACC:"));
  Serial.print(accelAngX, 2);
  Serial.print(F(", "));
  Serial.print(accelAngY, 2);
  Serial.print(F(", "));
  Serial.print(accelAngZ, 2);
  Serial.print(F("#GYR:"));
  Serial.print(gyroAngX, 2);
  Serial.print(F(", "));
  Serial.print(gyroAngY, 2);
  Serial.print(F(", "));
  Serial.print(gyroAngZ, 2);
  Serial.print(F("#FIL:"));
  Serial.print(complementX, 2);
  Serial.print(F(", "));
  Serial.print(complementY, 2);
  Serial.print(F(", "));
  Serial.print(complementZ, 2);
  Serial.println(F(" "));
  delay(5);
}
void getAcc()
{
  float accX, accY, accZ;
  float accXZ, accYZ;
  accX = acX - baseAcX;
  accY = acY - baseAcY;
  accZ = acZ - (16384 - baseAcZ);
  accYZ = sqrt(pow(accY, 2) + pow(accZ, 2));
  accelAngY = atan(-accX / accYZ) * 180 / PI;
  accXZ = sqrt(pow(accX, 2) + pow(accZ, 2));
  accelAngX = atan(accY / accXZ) * 180 / PI;
  accelAngZ = 0;
}
void getGyro()
{
  gyroX = (gyX - baseGyX) / 131;
  gyroY = (gyY - baseGyY) / 131;
  gyroZ = (gyZ - baseGyZ) / 131;
  gyroAngX += gyroX * 1;
  gyroAngY += gyroY * 1;
  gyroAngZ += gyroZ * 1;
}
void compementary()
{
  float tmp_angleX, tmp_angleY, tmp_angleZ;
  tmp_angleX = complementX + gyroX*dT;
  tmp_angleY = complementY + gyroY*dT;
  tmp_angleZ = complementZ + gyroZ*dT;
  complementX = 0.96*tmp_angleX + 0.04*accelAngX;
  complementY = 0.96*tmp_angleY + 0.04*accelAngY;
  complementZ = tmp_angleZ;
}
void initMotorSpeed()
{
  analogWrite(motorA,Throttle);
  analogWrite(motorB,Throttle);
  analogWrite(motorC,Throttle);
  analogWrite(motorD,Throttle);
  CheckMsp();
}
void CheckMsp()
{
  static uint32_t cnt; //unsigned int
  while (Serial1.available() > 0)
     {
    uint8_t mspData = Serial1.read();//usigned char
    if (mspData == '$')
      cnt = head1;
    else cnt++;
    msp[cnt] = mspData;
    if (cnt == crc)
      {
      if (msp[dataType] == 150)
         Throttle = msp[Throtte];
      }
    }
  
}


