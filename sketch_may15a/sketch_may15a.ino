#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <SoftwareSerial.h>
Servo MT1; Servo MT2; Servo MT3; Servo MT4;
double accelX = 0, accelY = 0, accelZ = 0, temp = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
double angleX = 0, angleY = 0, angleZ = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;
double FinalX = 0, FinalY = 0;
double PGain = 1.2, IGain = 2.5, DGain = 3.2;
double error_X = 0, error_Y = 0, error_Z=0, Pre_error_X, Pre_error_Y, Pre_error_Z;
double PX, IX, DX, PIDX = 0; double yaw, Pitch, roll;
double PY, IY, DY, PIDY = 0;
double PZ, IZ, DZ, PIDZ = 0;
double derised_angle = 0.0, current_angle_X, current_angle_Y;
SoftwareSerial BTSerial(0,1);
int spd = 0;
int MPU = 0x68;
int time;
void ControlSpd()
{
  if (Serial.available())
  {
    analogWrite(6, 255);
    analogWrite(5, 255);
    analogWrite(9, 255);
    analogWrite(10, 255);
  }
}
void PIDcal()
{
  PX = PGain * error_X;
  IX += IGain * 0.001 * error_X;
  DX = DGain * (error_X - Pre_error_X) / 0.001; //시간 0.001초
  Pre_error_X = error_X;
  PIDX = PX + IX + DX;
  PIDX = constrain(PIDX, 0, 255);

  PY = PGain * error_Y;
  IY = IY + IGain*0.001*error_Y;
  DY = DGain * (error_Y - Pre_error_Y) / 0.001;
  Pre_error_Y = error_Y;
  PIDY = PY + IY + DY;
  PIDY = constrain(PIDY, 0, 255);

  PZ = PGain * error_Z;
  IZ = IZ + IGain*0.001*error_Z;
  DZ = DGain * (error_Z - Pre_error_Z) / 0.001;
  Pre_error_Z = error_Z;
  PIDZ = PZ + IZ + DZ;
  PIDZ = constrain(PIDZ, 0, 255);
}
void setMotor()
{
  MT1.attach(6);
  MT2.attach(5);
  MT3.attach(9);
  MT4.attach(10);
}
void avalserial()
{
  if (BTSerial.available())
  {
    Serial.write(BTSerial.read());
  }
  else if (Serial.available())
  {
    BTSerial.write(Serial.read());
  }
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
  BTSerial.begin(115200);
  setMotor();
  setI2C();

}
void loop()
{
  avalserial();
  time = millis();
  gyrocalcul();
  ControlSpd();
}
//상보필터 
void Complementary()
{
  angleX = atan2(accelY, accelZ) * 180 / PI; //가속도 센서의 X구하기
  angleY = atan2(accelX, accelZ) * 180 / PI; //가속도 센서의 Y구하기
  angleZ = atan2(accelX, accelY) * 180 / PI; //가속도 센서의 Z구하기
  gyro_x = gyroX / 16383.0 * 90; //각속도 센서의 각도구하기
  gyro_y = gyroY / 16383.0 * 90; //각속도 센서의 각도구하기
  gyro_z = gyroZ / 16383.0 * 90;
    Pitch = (0.95*(Pitch + (gyro_x*0.001))) + (0.05*angleX);//상보 필터 구하기 0.95와 0.05는 더해서 1이 나와야한다. 
  roll = (0.95*(roll + (gyro_y*0.001))) + (0.05*angleY);//0.001의 시간으로 적분해준다.
  yaw = (0.95*(yaw + (gyro_z*0.001))) + (0.05*angleY);
  error_X = derised_angle - FinalX;
  error_Y = derised_angle - FinalY;
}
void gyrocalcul()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6);
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
  PIDcal();
 
  delay(1000);
}

