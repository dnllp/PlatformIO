#include <Arduino.h>

int sensor = 8;
int led = 25;
void setup() {
  pinMode(sensor, INPUT);
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}
void loop() {
 int valor = 0;
 valor = digitalRead(sensor);
 Serial.println(valor); 
 delay(500);
    digitalWrite(led, HIGH);
  delay(500);
    digitalWrite(led, LOW);
  
}
