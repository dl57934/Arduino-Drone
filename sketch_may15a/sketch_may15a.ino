#include <Servo.h>
#include <Wire.h>
#include <math.h>
Servo MT1;Servo MT2;Servo MT3;Servo MT4;
double accelX = 0, accelY = 0, accelZ = 0, temp = 0, gyroX=0, gyroY=0, gyroZ=0;
double angleX = 0, angleY = 0, angleZ = 0,gyro_x=0,gyro_y=0;
double FinalX = 0, FinalY = 0;
int spd=0;
int MPU = 0x68;
int time;
void ControlSpd()
{
  if (Serial.available())
  {
    char num = Serial.read();
    if (num == 'u')
      spd += 20;
    else if (num == 'd')
      spd -= 20;
    MT1.write(spd);
    MT2.write(spd);
    MT3.write(spd);
    MT4.write(spd);
  }
}
void setMotor()
{
  MT1.attach(6);
  MT2.attach(5);
  MT3.attach(9);
  MT4.attach(10);
}
void setI2C()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
}
void setup()
{
  Serial.begin(115200);
  setMotor();
  setI2C();
}
void loop()
{
  time = millis();
  ControlSpd();
  gyrocalcul();
}
//상보필터 
void Complementary()
{
  angleX = atan2(accelY,accelZ) * 180 / PI; //가속도 센서의 X구하기
  angleY = atan2(accelX,accelZ) * 180 / PI; //가속도 센서의 Y구하기
 gyro_x = gyroX / 16383.0 * 90; //각속도 센서의 각도구하기
 gyro_y = gyroY / 16383.0 * 90; //각속도 센서의 각도구하기
  FinalX = (0.95*(FinalX + (gyro_x*0.001))) + (0.05*angleX);//상보 필터 구하기 0.95와 0.05는 더해서 1이 나와야한다. 
  FinalY = (0.95*(FinalY + (gyro_y*0.001))) + (0.05*angleY);//0.001의 시간으로 적분해준다. 
}
void gyrocalcul()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
  Complementary();
  Serial.print("X값: "); Serial.println(FinalX);
  Serial.print("Y값: "); Serial.println(FinalY);
  delay(100);
}

