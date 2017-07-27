#include <OLLO.h>
OLLO myOLLO;

void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  myOLLO.begin(1);//DMS Module must be connected at port 1.
  myOLLO.begin(2);//DMS Module must be connected at port 2.
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(3, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  delay(500);
  digitalWrite(3, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  delay(500);
  digitalWrite(3, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  delay(500);    

  Serial.print("DMS Sensor 1 ADC Value = ");
  Serial.println(myOLLO.read(1)); //read ADC value from OLLO port 1
  delay(60);
  Serial.print("DMS Sensor 2 ADC Value = ");
  Serial.println(myOLLO.read(2)); //read ADC value from OLLO port 2
  delay(60);
}
