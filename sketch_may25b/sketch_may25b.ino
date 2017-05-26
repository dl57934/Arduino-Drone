#define Throttle_Max 255
#define Throttle_Min 0
#define Throttle_InI 5
enum { head1,head2,head3,
  dataSize, dataType,
  Roll,Pitch,Yaw,Throtte,Aux,
  crc,PackSize
};
int Throttle;
uint8_t msp[PackSize];
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
}
void loop()
{
  initMotorSpeed();
}
void checkMspPacket()
{
  static uint32_t cnt;
  while (Serial1.available() > 0)
  {
    uint8_t mspData = Serial1.read();
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


int motorA_Pin = 6;
int motorB_Pin = 10;
int motorC_Pin = 9;
int motorD_Pin = 5;
void initMotorSpeed()
{
  analogWrite(motorA_Pin, Throttle);
  analogWrite(motorB_Pin, Throttle_Min);
  analogWrite(motorC_Pin, Throttle);
  analogWrite(motorD_Pin, Throttle_Min);
  checkMspPacket();
}
