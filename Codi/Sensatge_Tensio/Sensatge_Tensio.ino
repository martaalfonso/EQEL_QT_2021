
#define sensatge A0

float battery_voltage;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //analogReadResolution(12);

}

void loop() {
  // put your main code here, to run repeatedly:
  battery_voltage = analogRead(sensatge);
  Serial.println(battery_voltage);

  battery_voltage = battery_voltage*3.1588493/1023;   //Valor máximo de tensión por el diseño actual

  delay(500);
  battery_voltage = (battery_voltage/0.391357273 + 1.2)/0.662251656;    //Antitransformada de la tensión de entrada al esp32, correspondiente a la tensión de bateria
  Serial.println(battery_voltage);
  delay(1000);

}
