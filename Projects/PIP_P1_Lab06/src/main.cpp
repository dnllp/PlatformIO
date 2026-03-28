#include <Arduino.h>
// LAB 6: Sensor HC-SR04 con LCD I2C 
// Instalar librerías: Wire.h (incluida) + LiquidCrystal_I2C 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
 
LiquidCrystal_I2C lcd(0x27, 16, 2); 
 
const int TRIG  = 9; 
const int ECHO  = 10; 
const int LED   = 13; 
const int BUZZ  = 8; 
const int ALERTA_CM = 20;  // Distancia de alerta 
 
long medirDistancia() { 
  digitalWrite(TRIG, LOW);  delayMicroseconds(2); 
  digitalWrite(TRIG, HIGH); delayMicroseconds(10); 
  digitalWrite(TRIG, LOW); 
  long dur = pulseIn(ECHO, HIGH, 30000); // Timeout 30ms 
  return dur == 0 ? -1 : (dur * 0.0343 / 2); 
} 
 
void setup() { 
  pinMode(TRIG, OUTPUT); 
  pinMode(ECHO, INPUT); 
  pinMode(LED,  OUTPUT); 
  pinMode(BUZZ, OUTPUT); 
  Serial.begin(9600); 
  lcd.init(); 
  lcd.backlight(); 
  lcd.print("HC-SR04 Listo!"); 
  delay(1500); 
} 
 
void loop() { 
  long dist = medirDistancia(); 
 
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print("Distancia:"); 
  lcd.setCursor(0, 1); 
 
  if (dist < 0) { 
    lcd.print("Sin objeto"); 
    digitalWrite(LED, LOW); 
    digitalWrite(BUZZ, LOW); 
  } else { 
    lcd.print(dist); lcd.print(" cm"); 
    Serial.println(dist); 
    bool alerta = (dist < ALERTA_CM); 
    digitalWrite(LED,  alerta); 
    digitalWrite(BUZZ, alerta); 
  } 
  delay(300); 
}