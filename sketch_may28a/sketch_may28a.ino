enum {
  head1, head2, head3,
  size, type, 
  roll,pitch,yaw,
  throtte,aux,crc
};
int Thro;
int motorA = 5, motorB = 6, motorC = 9, motorD = 10;
uint8_t masp[11];
int data;
uint32_t cnt;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  /* add setup code here */

}

void loop()
{
  controlSpeed();
}
void checkMasp()
{
  static uint32_t cnt;
  if(Serial1.available()>0)
  {
    while(Serial1.available()>0)
    {
      uint8_t mspdata=Serial1.read();
      if(mspdata = '$') cnt = head1;
      else cnt++;
      masp[cnt] = mspdata;
      if(cnt == crc)
      {
        if(masp[type] == 150)
        {
        Thro = masp[throtte];
        }
      }
    }
  }
}
void updateMotorSpeed()
{
  analogWrite(5,Thro);
  analogWrite(6,Thro);
  analogWrite(9,Thro);
  analogWrite(10,Thro);
}
void controlSpeed()
{
  checkMasp();
  updateMotorSpeed();
}

