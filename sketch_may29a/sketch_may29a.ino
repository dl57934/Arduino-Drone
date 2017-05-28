#include <Wire.h>
int MPU = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float dT;
//float roll_target_angle=0.0, roll_kp=1, roll_ki=0, roll_kd=0, roll_iterm;

//float pitch_prev_angle=0.0, pitch_kp=1, pitch_ki=0, pitch_kd=0, pitch_iterm;
//float  yaw_prev_angle=0.0, yaw_kp=1, yaw_ki=0, yaw_kd=0, yaw_iterm;
float gyro_x, gyro_y, gyro_z;
float gyro_xz, gyro_yz;
float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 1;
float roll_stabilize_ki = 0;
float roll_rate_kp = 1;
float roll_rate_ki = 0;
float roll_stabilize_iterm;
float roll_rate_itrem;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 1;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 1;
float pitch_rate_ki = 0;
float pitch_stabilize_iterm;
float pitch_rate_itrem;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 1;
float yaw_rate_ki = 0;
float yaw_stabilize_iterm;
float yaw_rate_itrem;
float yaw_output;
enum
{
  head1, head2, head3,
  sizex, type, roll, pitch, yaw, thro, aux, crc
};
void setup()
{

  Serial.begin(115200);
  Serial1.begin(115200);
  SetI2c();
  getInitavg();
  InitDt();
  InitYPR();
  InitSpeed();
  /* add setup code here */
}

void loop()
{
  getAngle();
  calDt();
  calAccel();
  calGyro();
  complementary();
  //calcYPTtoStdPID();
  calcYPRtoDualPID();
  calcMotorSpeed();
  checkMspPacket();
  InitSpeed();
  updateSpeed();
  /*static int cnt;
  cnt++;
  if (cnt % 2 == 0)
  SendDataToProcessing();*/
  /* add main program code here */

}
void getAngle()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
void getInitavg()
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  for (int i = 0; i < 10; i++)
  {
    getAngle();
    sumAcX += accelX; sumAcY += accelY; sumAcZ += accelZ;
    sumGyX += gyroX; sumGyY += gyroY; sumGyZ += gyroZ;
    delay(100);
  }
  baseAcX = sumAcX / 10;    baseAcY = sumAcY / 10;    baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10;   baseGyY = sumGyY / 10;   baseGyZ = sumGyZ / 10;
}
unsigned long Tnow, Tprev;
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
  accel_z = accelZ + (16384 - baseAcZ);
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
 extern float roll_target_angle , pitch_target_angle , yaw_target_angle ;
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

/*void calcYPTtoStdPID()
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
}*/
float throttle = 0;
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;
void calcMotorSpeed()
{
  motorA_speed = (throttle == 0) ? 0 : throttle + yaw_output + roll_output + pitch_output;
  motorB_speed = (throttle == 0) ? 0 : throttle - yaw_output - roll_output + pitch_output;
  motorC_speed = (throttle == 0) ? 0 : throttle + yaw_output - roll_output - pitch_output;
  motorD_speed = (throttle == 0) ? 0 : throttle - yaw_output + roll_output - pitch_output;
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
/*void SendDataToProcessing()
{
Serial.print(F("DEL:"));
Serial.print(dT, DEC);
Serial.print(F("#RPY:"));
Serial.print(complememtX, 2);
Serial.print(F(", "));
Serial.print(complememtY, 2);
Serial.print(F(", "));
Serial.print(complememtZ, 2);
Serial.print(F("finalGyroZ"));
Serial.print(finalGyroZ, 2);
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
}*/
uint8_t mspPacket[11];
void checkMspPacket()
{
  static uint32_t cnt;

  if (Serial1.available()>0)
  {
    while (Serial1.available()>0)
    {
      uint8_t inputdata = Serial1.read();
      if (inputdata == '$')
        cnt = head1;
      else cnt++;
      mspPacket[cnt] = inputdata;
      if (cnt == crc)
      {
        if (mspPacket[type] == 150)
        {
          throttle = mspPacket[thro];
        }
      }
    }
  }
}
int motorA = 6;
int motorB = 10;
int motorC = 9;
int motorD = 5;
void InitSpeed()
{
  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
  analogWrite(motorC, 0);
  analogWrite(motorD, 0);
}
void updateSpeed()
{
  analogWrite(motorA, motorA_speed);
  analogWrite(motorB, motorB_speed);
  analogWrite(motorC, motorC_speed);
  analogWrite(motorD, motorD_speed);
}
void DualPID(float target_angle,
  float angle_in,
  float rate_in,
  float stabilize_kp,
  float stabilize_ki,
  float rate_kp,
  float rate_ki,
  float &stabilize_iterm, float &rate_iterm, float&output)
{
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;
  angle_error = target_angle - angle_in;
  stabilize_pterm = stabilize_kp *angle_error;
  stabilize_iterm += stabilize_ki*angle_error*dT;
  desired_rate = stabilize_pterm;
  rate_error = desired_rate - rate_in;
  rate_pterm = rate_kp*rate_error;
  rate_iterm += rate_ki*rate_error*dT;
  output = rate_pterm + rate_iterm + stabilize_iterm;
}

void calcYPRtoDualPID()
{
  roll_angle_in = complememtY;
  roll_rate_in = gyro_y;
  DualPID(roll_target_angle, roll_angle_in, roll_rate_in, roll_stabilize_kp, roll_stabilize_ki, roll_rate_kp, roll_rate_ki, roll_stabilize_iterm, roll_rate_itrem, roll_output);
  pitch_angle_in = complememtX;
  pitch_rate_in = gyro_x;
  DualPID(pitch_target_angle, pitch_angle_in, pitch_rate_in, pitch_stabilize_kp, pitch_stabilize_ki, pitch_rate_kp, pitch_rate_ki, pitch_stabilize_iterm, pitch_rate_itrem, pitch_output);
  yaw_angle_in = complememtZ;
  yaw_rate_in = gyro_z;
  DualPID(yaw_target_angle, yaw_angle_in, yaw_rate_in,yaw_stabilize_kp, yaw_stabilize_ki, yaw_rate_kp, yaw_rate_ki, yaw_stabilize_iterm, yaw_rate_itrem, yaw_output);


}
