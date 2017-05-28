#include <Wire.h>
int MPU = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float dT;
float roll_prev_angle=0.0, roll_kp=1, roll_ki=0, roll_kd=0, roll_iterm;
float roll_output, pitch_output, yaw_output;
float pitch_prev_angle=0.0, pitch_kp=1, pitch_ki=0, pitch_kd=0, pitch_iterm;
float  yaw_prev_angle=0.0, yaw_kp=1, yaw_ki=0, yaw_kd=0, yaw_iterm;
float gyro_x, gyro_y, gyro_z;
float gyro_xz, gyro_yz;
unsigned long Tnow, Tprev;
void setup()
{
  SetI2c();
  Serial.begin(115200);
  Serial1.begin(115200);
  getInitavg();
  InitDt();
  InitYPR();
  /* add setup code here */
}

void loop()
{
  getAngle();
  calDt();
  calAccel();
  calGyro();
  complementary();
  calcYPTtoStdPID();
  calcMotorSpeed();
  static int cnt;
  cnt++;
  if (cnt % 2 == 0)
    SendDataToProcessing();
  /* add main program code here */

}
void getAngle()
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
}
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
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
  dT = (Tnow - Tprev) / 1000000.0;
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
float finalAccelX, finalAccelY, finalAccelZ;
float finalGyroX, finalGyroY, finalGyroZ;
void calAccel()
{

  accel_x = accelX - baseAcX;
  accel_y = accelY - baseAcY;
  accel_z = accelZ - (16384 - baseAcZ);
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  finalAccelY = atan(-accel_x / accel_yz) * 180 / PI;
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  finalAccelX = atan(accel_y / accel_xz) * 180 / PI;
  finalAccelZ = 0;
}
void calGyro()
{

  gyro_x = (gyroX - baseGyX) / 131;
  gyro_y = (gyroY - baseGyY) / 131;
  gyro_z = (gyroZ - baseGyZ) / 131;
  finalGyroX += gyro_x*dT;
  finalGyroY += gyro_y*dT;
  finalGyroZ += gyro_z*dT;
}
float complememtX, complememtY, complememtZ;
void complementary()
{
  float tmpX, tmpY, tmpZ;
  tmpX = complememtX + gyro_x*dT;
  tmpY = complememtY + gyro_y*dT;
  tmpZ = complememtZ + gyro_z*dT;
  complememtX = 0.96*tmpX + 0.04*finalAccelX;
  complememtY = 0.96*tmpY + 0.04*finalAccelY;
  complememtZ = tmpZ;
}
float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;
float roll_target_angle=0.0, pitch_target_angle=0.0, yaw_target_angle=0.0;
void  InitYPR()
{
  for (int i = 0; i < 10; i++)
  {
    getAngle();
    calDt();
    calAccel();
    calGyro();
    complementary();
    base_roll_target_angle += complememtY;
    base_pitch_target_angle += complememtX;
    base_yaw_target_angle += complememtZ;
    delay(100);
  }
  base_roll_target_angle /= 10;
  base_pitch_target_angle /= 10;
  base_yaw_target_angle /= 10;
  roll_target_angle = base_roll_target_angle;
  pitch_target_angle = base_pitch_target_angle;
  yaw_target_angle = base_yaw_target_angle;
}

void calcYPTtoStdPID()
{
  stdPID(roll_target_angle, complememtY, roll_prev_angle, roll_kp, roll_ki, roll_kd, roll_iterm, roll_output);
  stdPID(pitch_target_angle, complememtX, pitch_prev_angle, pitch_kp, pitch_ki, pitch_kd, pitch_iterm, pitch_output);
  stdPID(yaw_target_angle, complememtZ, yaw_prev_angle, yaw_kp, yaw_ki, yaw_kd, yaw_iterm, yaw_output);
}
void stdPID(float& setpoint, float& input, float& prev_input, float& kp, float& ki, float& kd, float& iterm, float& output)
{
  float error;
  float dInput;
  float pterm, dterm;
  error = setpoint - input;
  dInput = input - prev_input;
  prev_input = input;
  pterm = kp*error;
  iterm += ki*error*dT;
  dterm = -kd*dInput / dT;
  output = pterm + iterm + dterm;
}
float throttle = 100;
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;
void calcMotorSpeed()
{
  motorA_speed = throttle + yaw_output + roll_output + pitch_output;
  motorB_speed = throttle - yaw_output - roll_output + pitch_output;
  motorC_speed = throttle + yaw_output - roll_output - pitch_output;
  motorD_speed = throttle - yaw_output + roll_output - pitch_output;
  if (motorA_speed < 0)
    motorA_speed = 0;
  if (motorA_speed>255)
    motorA_speed = 255;
  if (motorB_speed < 0)
    motorB_speed = 0;
  if (motorB_speed > 255)
    motorB_speed = 255;
  if (motorC_speed < 0)
    motorC_speed = 0;
  if (motorC_speed>255)
    motorC_speed = 255;
  if (motorD_speed < 0)
    motorD_speed = 0;
  if (motorD_speed > 255)
    motorD_speed = 255;
}
void SendDataToProcessing()
{
  Serial.print(F("DEL:"));
  Serial.print(dT, DEC);
  Serial.print(F("#RPY:"));
  Serial.print(complememtX, 2);
  Serial.print(F(", "));
  Serial.print(complememtY, 2);
  Serial.print(F(", "));
  Serial.print(complememtZ, 2);
  Serial.print(F("#PID:"));
  Serial.print(roll_output, 2);
  Serial.print(F(", "));
  Serial.print(pitch_output, 2);
  Serial.print(F(", "));
  Serial.print(yaw_output, 2);
  Serial.print(F("#A:"));
  Serial.print(motorA_speed, 2);
  Serial.print(F("#B: "));
  Serial.print(motorB_speed, 2);
  Serial.print(F("#C: "));
  Serial.print(motorC_speed, 2);
  Serial.print(F("#D: "));
  Serial.println(motorD_speed, 2);
  delay(100);
}

