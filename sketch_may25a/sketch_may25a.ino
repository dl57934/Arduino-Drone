enum { head1,head2,head3,
  dataSize, dataType,
  Roll,Pitch,Yaw,Throtte,Aux,
  crc,PackSize
};
uint8_t msp[PackSize];
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
}
void loop()
{
checkMspPacket();
}
void checkMspPacket()
{
  static uint32_t cnt;
  if(Serial1.available() > 0)
  {
  while (Serial1.available() > 0)
  {
    uint8_t mspData = Serial1.read();
    if (mspData == '$')
      cnt = head1;
    else cnt++;
    msp[cnt] = mspData;
    if (cnt == crc)
      printMspPacket();
  }
}
}
void printMspPacket()
{
  Serial.print((char)msp[head1]);
  Serial.print((char)msp[head2]);
  Serial.print((char)msp[head3]);
  Serial.print(msp[dataSize]);
  Serial.print(" ");
  Serial.print(msp[dataType]);
  Serial.print(" ");
  Serial.print("R  ");
  Serial.print(msp[Roll]);
  Serial.print("P  ");
  Serial.print(msp[Pitch]);
  Serial.print("Y  ");
  Serial.print(msp[Yaw]);
  Serial.print("T  ");
  Serial.print(msp[Throtte]);
  Serial.print("A  ");
  Serial.print(msp[Aux]);
  Serial.print("crc  ");
  Serial.print(msp[crc]);
  Serial.print("\n");
}
