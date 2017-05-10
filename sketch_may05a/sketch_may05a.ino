#include <SoftwareSerial.h>
// SoftwareSerial(RX, TX) 형식으로 블루투스 모듈과 교차하여 연결된다.
SoftwareSerial BTSerial(0, 1);
void setup()
{
Serial.begin(9600);
BTSerial.begin(9600); // 블루투스 모듈 통신 속도의 디폴트값
}
void loop()
{
if (BTSerial.available()) // 블루투스 모듈에서 수신된 데이터를 컴퓨터로
Serial.write(BTSerial.read());
if (Serial.available()) // 컴퓨터에서 수신된 데이터를 블루투스 모듈로
BTSerial.write(Serial.read());
}
