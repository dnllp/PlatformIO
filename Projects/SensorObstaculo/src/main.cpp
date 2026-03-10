#include <Arduino.h>

int sensor = 8;

void setup() {
  pinMode(sensor, INPUT);
  Serial.begin(9600);
}
void loop() {
 int valor = 0;
 valor = digitalRead(sensor);
 Serial.println(valor); 
 delay(50);
}
