int buttonPin =7;
int ledPin = 13;
int val=0;
void setup() {
  // put your setup code here, to run once:
 pinMode(buttonPin,INPUT_PULLUP);
 pinMode(ledPin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
val=digitalRead(buttonPin);
digitalWrite(ledPin,val);
}
