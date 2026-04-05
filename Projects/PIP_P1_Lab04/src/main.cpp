// LAB 4: Sistema de Alarma PIR con Buzzer 
#include <Arduino.h> 
const int PIR_PIN   = 2;   // INT0 - Interrupción externa 
const int BUZZ_PIN  = 8; 
const int LED_PIN   = 13; 
const int ALARMA_MS = 3000; // Duración alarma (ms) 
 
volatile bool movimiento = false; // Flag de interrupción 
unsigned long tiempoAlarma = 0; 
 
// ISR: Rutina de Servicio de Interrupción 
void ISR_Movimiento() { 
  movimiento = true; 
  tiempoAlarma = millis(); 
  Serial.println("[INTERRUPCION] Movimiento detectado!"); 
} 
 
void setup() { 
  pinMode(PIR_PIN,  INPUT); 
  pinMode(BUZZ_PIN, OUTPUT); 
  pinMode(LED_PIN,  OUTPUT); 
  Serial.begin(9600); 
  Serial.println("Sistema de Alarma PIR listo..."); 
 
  // Adjuntar ISR al pin 2 (INT0), flanco de subida 
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), 
                  ISR_Movimiento, RISING); 
} 
 
void loop() { 
  if (movimiento) { 
    digitalWrite(LED_PIN, HIGH); 
    digitalWrite(BUZZ_PIN, HIGH); 
 
    if (millis() - tiempoAlarma > ALARMA_MS) { 
      movimiento = false; 
      digitalWrite(LED_PIN, LOW); 
      digitalWrite(BUZZ_PIN, LOW); 
      Serial.println("Alarma desactivada."); 
    } 
  } 
  delay(50); 
} 