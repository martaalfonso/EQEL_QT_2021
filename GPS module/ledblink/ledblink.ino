#define led 21
void setup() {
  // put your setup code here, to run once:
pinMode(21,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(led,HIGH);
delay(1000);
digitalWrite(led,LOW);
delay(1000);
}
