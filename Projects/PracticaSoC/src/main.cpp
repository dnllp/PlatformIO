#include <Arduino.h>

int led = 2;

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(led,HIGH);
  Serial.println("Encendido");
  delay(500);
  digitalWrite(led,LOW);
  Serial.println("Apagado");
  delay(500);
}

