#include <Arduino.h>
int led =D4;
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  pinMode(led,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led,HIGH);
  delay(500);
  digitalWrite(led,LOW);
  delay(500);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}